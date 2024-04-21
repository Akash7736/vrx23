#!/usr/bin/env python3
from rclpy.node import Node
import rclpy
import numpy as np
from geometry_msgs.msg import PoseArray, Pose
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64, Float64MultiArray
from vrx_sub.utils import euler_from_quaternion, change_range, global_to_local_rot
from vrx_sub.config import *
from std_srvs.srv import Trigger

# class WayPointSorter:
#     def __init__(self, waypoints: np.ndarray) -> None:
#         self.waypoints = waypoints
#         self.min_dist = np.inf
#         self.arr = range(0, waypoints.shape[0])

#     def permutations(self, waypoints: np.ndarray, r=[]):
#         output = waypoints.tolist()
#         arr = range(0, output.shape[0])
#         min_dist = np.inf
#         if not arr:
#             return r
#         else:
#             first = arr[0]
#             for i in range(len(r) + 1):
#                 self.permutations(arr[1:], r[:i] + [first] + r[i:])


class GlobalPathPlanner(Node):
    def __init__(self):
        super().__init__('GlobalPlanner')

        self.waypoint_sub = self.create_subscription(
            PoseArray, '/wamv/waypoints', self.waypoint_callback, 10)
        self.kf_sub = self.create_subscription(
            Odometry, '/wamv/kf/state', self.odom_callback, 10)

        self.desired_state_pub = self.create_publisher(
            Float64MultiArray, '/wamv/desired_state', 10)
        self.loop = self.create_timer(
            timer_period_sec=0.1, callback=self.compute_target_state
        )

        self.jump_service = self.create_service(Trigger, '/wamv/path_planning/next_waypoint', self.next_waypoint_callback)
        self.waypoints = None
        self.waypoint_idx = 0
        self.pose = None
        self.apf = APF(K_Att=1, K_Critical_Rep=10, K_Rep=2, Eta=0.01, rho_0=5)

        self.dist_error_thr = 0.01
        self.yaw_error_thr = 0.0001
        self.error_thr = 0.04

        self.vel_control_thr = np.inf
        self.net_vel_mag = 1

        self.force_vec_mag = 1

    def next_waypoint_callback(self, msg: Trigger):
        self.waypoint_idx = min(self.waypoint_idx + 1, len(self.waypoints) - 1)

    def odom_callback(self, msg: Odometry):
        if self.pose is None:
            self.pose = Pose()
        self.pose = msg.pose.pose

    def compute_target_state(self):
        if self.pose is None:
            print('No Pose')
            return
        if self.waypoints is None:
            print('No Waypoints')
            return
        pose = np.array([self.pose.position.x, self.pose.position.y])
        _, _, yaw = euler_from_quaternion(self.pose.orientation)
        global_to_local_mat = global_to_local_rot(yaw)[:2, :2]

        self.waypoint_idx = min(self.waypoint_idx, len(self.waypoints) - 1)

        if len(self.waypoints) - 1 == self.waypoint_idx:
            print('******* Last Point ********')

        desired_pose, desired_yaw = self.waypoints[self.waypoint_idx][:
                                                                      2], self.waypoints[self.waypoint_idx][2]
        dist_error = np.linalg.norm(desired_pose - pose)
        yaw_error = change_range(desired_yaw - yaw)

        error = dist_error + ((0.75) ** dist_error) * np.abs(yaw_error)

        desired_state = [MODE_PID_POSE, desired_pose[0],
                         desired_pose[1], desired_yaw, 0, 0]
        

        print('Pose :', pose)
        print('YAW :', yaw)
        print('Desired Pose:', desired_pose)
        print('Dist Error: ', dist_error)
        print('Error :', error)

        if error < self.error_thr:
            self.waypoint_idx += 1
            return

        if dist_error > self.vel_control_thr and False:
            desired_global_vel_grad = self.apf.velocity_gradient(
                pose=pose, goal=desired_pose, obstacles=None)
            theta_vel = np.arctan2(
                desired_global_vel_grad[1], desired_global_vel_grad[0])
            desired_force = self.force_vec_mag * desired_global_vel_grad
            desired_state = [MODE_PID_FORCE, desired_force[0], desired_force[1], theta_vel,
                             0, 0]

        if dist_error > self.vel_control_thr and False:
            desired_global_vel_grad = self.apf.velocity_gradient(
                pose=pose, goal=desired_pose, obstacles=None)

            print('GV: ', desired_global_vel_grad)
            theta_vel = np.arctan2(
                desired_global_vel_grad[1], desired_global_vel_grad[0])
            desired_vel = self.net_vel_mag * \
                (global_to_local_mat @ desired_global_vel_grad)
            desired_state = [MODE_PID_VEL, 0, 0, theta_vel,
                             desired_vel[0], desired_vel[1]]

        if dist_error > self.vel_control_thr:
            desired_global_vel_grad = self.apf.velocity_gradient(
                pose=pose, goal=desired_pose, obstacles=None)
            theta_vel = np.arctan2(
                desired_global_vel_grad[1], desired_global_vel_grad[0])

            desired_vel = self.net_vel_mag * (desired_global_vel_grad)
            desired_state = [1, pose[0] + desired_vel[0], pose[1] + desired_vel[1], theta_vel,
                             0, 0]

        desired_state_msg = Float64MultiArray()
        desired_state_msg.data.fromlist(desired_state)
        # desired_state_msg.data = desired_state
        print(desired_state_msg)
        self.desired_state_pub.publish(desired_state_msg)

    def waypoint_callback(self, msg: PoseArray):
        waypoints = []
        for pose in msg.poses:
            _, _, yaw = euler_from_quaternion(pose.orientation)
            waypoints.append([pose.position.x, pose.position.y, yaw])

        self.waypoints = np.array(waypoints, dtype=np.float32)
        self.waypoints_idx = 0

class APF():
    def __init__(self, K_Att, K_Rep, K_Critical_Rep, Eta, rho_0) -> None:
        self.rho_0 = rho_0
        self.K_Att = K_Att
        self.K_Critical_Rep = K_Critical_Rep
        self.K_Rep = K_Rep
        self.Eta = Eta

    def cluster_to_obstacle(self, cluster, pose):
        rel = cluster - pose
        dist = np.linalg.norm(rel, axis=1)
        nearest_object_idx = np.argmin(dist)
        return cluster[nearest_object_idx]

    def velocity_gradient(self, pose, obstacles, goal):
        dist = np.linalg.norm(goal - pose)
        F_att = self.K_Att * (goal - pose)

        if obstacles is None:
            return F_att / np.linalg.norm(F_att)
        rel_obstacles = obstacles - pose
        rho = np.linalg.norm(rel_obstacles, axis=1)
        mask = rho < self.rho_0

        critical_obstacles = rel_obstacles[mask]
        rho_critical = rho[mask].reshape(-1, 1)
        distant_obstacles = rel_obstacles[~mask]
        rho_distant = rho[~mask].reshape(-1, 1)

        F_Critical_Rep_indv = (critical_obstacles) * -self.K_Critical_Rep * \
            ((1 / rho_critical) - (1/self.rho_0)) * ((1/(rho_critical**2)))

        F_Distant_Rep_indv = (distant_obstacles) * -self.K_Rep * \
            ((1 / rho_distant) - (1/self.rho_0)) * ((1/(rho_distant**2)))

        F_Rep = np.sum(F_Distant_Rep_indv, axis=0) + \
            np.sum(F_Critical_Rep_indv, axis=0)

        delta = self.Eta * (self.rho_0 - rho_critical)
        F_Vor_indv = delta * \
            np.array([(critical_obstacles[:, 1]), -
                     (critical_obstacles[:, 0])]).T
        F_Vor = np.sum(F_Vor_indv, axis=0)
        F_Total = F_att + F_Rep + F_Vor

        vel_grad = F_Total / np.linalg.norm(F_Total)

        return vel_grad


def main(args=None):
    rclpy.init(args=args)
    planner = GlobalPathPlanner()
    rclpy.spin(planner)
    planner.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()