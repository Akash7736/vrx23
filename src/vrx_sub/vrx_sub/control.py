#!/usr/bin/env python3
from rclpy.node import Node
import rclpy
from std_msgs.msg import Float64, Float64MultiArray
from nav_msgs.msg import Odometry
import numpy as np
from vrx_sub.controller_gains import *
from vrx_sub.utils import euler_from_quaternion, change_range, global_to_local_rot, quaternion_to_rotation_matrix, inv_glf
from vrx_sub.config import *


class PID:
    def __init__(self, gains,  intergral_fnx=lambda x: x * 0.0, rate=1):
        self.kp = gains.kp
        self.ki = gains.ki
        self.kd = gains.kd
        self.scale = gains.scale
        self.prev_error = 0
        self.integral = 0
        self.rate = rate

        self.file_name = f"x_pose.csv"

        with open(self.file_name, 'w') as fd:
            data = ",".join(map(str, ['eX', 'eY', 'eYAW', 'eVx', 'eVy', 'iEX', 'iEY',
                            'iEYAW', 'iEVx', 'iEVy', 'dEX', 'dEY', 'dEYAW', 'dEVx', 'dEVy']))
            fd.write(data+"\n")

        self.intergral_fnx = intergral_fnx

    def reset(self):
        self.prev_error = self.prev_error * 0
        self.integral = self.integral * 0

    def compute(self, error, derivative=None, integral=None):
        if integral is None:
            self.integral += self.intergral_fnx(error) * self.rate

        if derivative is None:
            derivative = (error - self.prev_error) * self.rate
        print('Error: ', error)
        print('P Forces :', self.kp @ error)
        print('I Force :', self.ki @ self.integral)
        print('D Force :', self.kd @ derivative)
        print('Integral :', self.integral)
        output = self.kp @ error + self.ki @ self.integral + self.kd @ derivative
        with open(self.file_name, 'a') as fd:
            data = ",".join(
                map(str, [error.tolist()+self.integral.tolist()+derivative.tolist()]))[1:-1]
            fd.write(data+"\n")
        self.prev_error = error
        return output


class Controller(Node):
    def __init__(self) -> None:
        super().__init__('controller')
        self.aft_left_pub = self.create_publisher(
            Float64, '/wamv/thrusters/leftaft/thrust', 10)
        self.aft_right_pub = self.create_publisher(
            Float64, '/wamv/thrusters/rightaft/thrust', 10)
        self.front_left_pub = self.create_publisher(
            Float64, '/wamv/thrusters/leftfore/thrust', 10)
        self.front_right_pub = self.create_publisher(
            Float64, '/wamv/thrusters/rightfore/thrust', 10)
        self.kf_sub = self.create_subscription(
            Odometry, '/wamv/kf/state', self.odom_callback, 10)
        self.goal_sub = self.create_subscription(
            Float64MultiArray, '/wamv/desired_state', self.desired_callback, 10)
        self.odom = Odometry()

        # Mode, X, Y, YAW, VX, VY
        # Mode => Pose Control (1)
        # Mode => Velocity Control (2)
        # Mode => Pose + Velocity Control (3)

        self.pid_rate = 10
        self.timer = self.create_timer(
            timer_period_sec=(1/self.pid_rate), callback=self.control)
        self.prev_gps, self.curr_gps = None, None

        if GLF_MODE:
            pose_gains = Pose_Gain_GLF()
            force_grains = Force_Gain_GLF()
        else:
            pose_gains = Pose_Gain()
            force_grains = Force_Gain()

        def fxn(x):
            return 1 - 2*(1/(1+np.exp(x)))

        self.pid_controllers = {
            MODE_PID_POSE: PID(rate=self.pid_rate, gains=pose_gains, intergral_fnx=lambda x: np.array([0.0, 0.0, x[2], 0.0, 0.0])),
            MODE_PID_FORCE: PID(rate=self.pid_rate, gains=force_grains)
        }

        # self.pid_all = PID(
        #     kp=config.kp_all, kd=config.kd_all, ki=config.ki_all)

        # self.pid_pose_control = PID(kp=0, kd=0, ki=0)
        # self.pid_vel_control = PID(kp=5, kd=0, ki=0.1)
        # self.pid_yaw_control = PID(kp=10, kd=-2, ki=0.0)

        self.TA = np.array([

            [0.707, 0.707, 0.707, 0.707],
            [-0.707, 0.707, -0.707, 0.707],
            [-1.626, 1.626, 1.626, -1.626]
        ])
        self.invTA = np.linalg.pinv(self.TA)

        print(self.invTA)

        self.desired_pose = np.array([0.0, 0.0])
        self.desired_vel = np.array([0.0, 0.0])
        self.desired_yaw = 0
        self.desired_mode = -1
        self.scale_factor = 1

        self.triggered = False

    def odom_callback(self, msg: Odometry):
        self.odom = msg
        self.triggered = True

    def desired_callback(self, msg: Float64MultiArray):
        self.desired_mode, x, y, self.desired_yaw, vx, vy = [
            x for x in msg.data]
        self.desired_mode = int(self.desired_mode)
        self.desired_pose[0], self.desired_pose[1] = x, y
        self.desired_vel[0], self.desired_vel[1] = vx, vy

    def publish_thrust(self, thrust_list=[0.0, 0.0, 0.0, 0.0], scale=1):
        thrust_list = np.array([inv_glf(x) for x in thrust_list]) * scale

        print('Command Values :', thrust_list)
        if np.isnan(thrust_list).any():
            thrust_list = [0.0, 0.0, 0.0, 0.0]
        self.front_left_pub.publish(Float64(data=thrust_list[0]))
        self.front_right_pub.publish(Float64(data=thrust_list[1]))
        self.aft_right_pub.publish(Float64(data=thrust_list[2]))
        self.aft_left_pub.publish(Float64(data=thrust_list[3]))

    def vel_control(self, desired, state, controller: PID):
        return controller.compute(error=desired - state)

    def control(self):
        if not self.triggered:
            return
        self.triggered = False

        pose = np.array([self.odom.pose.pose.position.x,
                        self.odom.pose.pose.position.y])
        _, _, yaw = euler_from_quaternion(self.odom.pose.pose.orientation)

        print('GLF Mode =>', GLF_MODE)
        print('Desried Mode:', self.desired_mode)
        print('Pose:', pose, self.desired_pose)
        print('Yaw:', yaw)
        print(
            f'Vel: [{self.odom.twist.twist.linear.x},{self.odom.twist.twist.linear.y}]', self.desired_vel)
        # print('Rot Mat', global_to_local_rot(
        #     yaw)[:2, :2])

        local_pose_error = global_to_local_rot(
            yaw)[:2, :2] @ (self.desired_pose - pose)
        print('Pose Error :', local_pose_error)
        error = np.array([
            local_pose_error[0],
            local_pose_error[1],
            change_range(self.desired_yaw - yaw),
            self.desired_vel[0] - self.odom.twist.twist.linear.x,
            self.desired_vel[1] - self.odom.twist.twist.linear.y
        ])
        if self.desired_mode in self.pid_controllers:
            forces = self.pid_controllers[self.desired_mode].compute(
                error=error)
            thrusts = self.scale_factor * self.invTA @ forces
            print('Thrust  Forces:', thrusts)
            self.publish_thrust(
                thrust_list=thrusts, scale=self.pid_controllers[self.desired_mode].scale)
        else:
            self.publish_thrust()


def main(args=None):
    rclpy.init(args=args)
    controller = Controller()
    rclpy.spin(controller)
    controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
