#!/usr/bin/env python3
from rclpy.context import Context
from rclpy.node import Node
import numpy as np
import rclpy
from nav_msgs.msg import Odometry
from ros_gz_interfaces.msg import ParamVec

from geometry_msgs.msg import PoseStamped, PoseArray, Pose
from vrx_sub.utils import euler_from_quaternion, gps_pose_to_pose
from vrx_sub.config import DATUM,GPS_WAMV_LINK_TO_BASE_LINK_TF

import itertools


class MissionPlanner(Node):
    def __init__(self):
        super().__init__('MissionPlanner')

        self.task_solve_df = {
            'stationkeeping': StationKeeping(self),
            'wayfinding': WayFinding(self),
            'perception': 0,
            'acoustic_perception': 0,
            'wildlife': 0,
            'gymkhana': 0,
            'acoustic_tracking': 0,
            'scan_dock_deliver': 0
        }

        self.task_info_sub = self.create_subscription(
            ParamVec, '/vrx/task/info', self.callback_task, 10)
        self.loop = self.create_timer(
            timer_period_sec=0.1, callback=self.solve)
        self.task_obj = None
        self.task_info_df = {}
    def callback_task(self, msg: ParamVec):
        for param in msg.params:
            self.task_info_df[param.name] = param.value

    def solve(self):
        if not 'name' in self.task_info_df:
            print('No task Info')
            return
        task_name = self.task_info_df['name'].string_value
        task_score = self.task_info_df['score'].double_value
        task_timeout = self.task_info_df['timed_out'].double_value
        task_state = self.task_info_df['state'].string_value
        print('Task Name :', task_name)
        task_obj = self.task_solve_df[task_name]
        print('Task Score :', task_score)
        print('Task Timeout :', task_timeout)
        print('Task State :',task_state)
        if task_timeout == True:
            print('...........Timeout.............')
            return
        task_obj.compute(task_score, task_timeout, task_state)

class WayFinding():
    def __init__(self, node: MissionPlanner) -> None:
        self.waypoint_sub = node.create_subscription(PoseArray, '/vrx/wayfinding/waypoints', self.callback_waypoint, 10)
        self.waypoint_pub = node.create_publisher(
            PoseArray, '/wamv/waypoints', 10)
        self.kf_sub = node.create_subscription(
            Odometry, '/wamv/kf/state', self.odom_callback, 10)
        self.way_points = None
        self.best_way = None
        self.pose = None

    def odom_callback(self, msg: Odometry):
        self.pose = np.array([msg.pose.pose.position.x,msg.pose.pose.position.y])

    def find_way(self, way_points: PoseArray, my_pose: np.ndarray):
        waypoints = []
        for pose in way_points.poses:
            pose: Pose
            waypoints.append([pose.position.x, pose.position.y])
        waypoints = np.array(waypoints, dtype = np.float32)
        permu = list(itertools.permutations([x for x in range(len(waypoints))]))
        dist_matrix = []
        for point in waypoints:
            dist_matrix.append(np.linalg.norm(waypoints - point, axis = 1))
        dist_matrix = np.array(dist_matrix)

        min_dist = 1e5
        min_path = permu[0]
        for path in permu:
            l_path = len(path)
            dist = np.linalg.norm(waypoints[path[0]] - my_pose)
            for i in range(l_path - 1):
                dist += dist_matrix[i][i+1]
            if dist < min_dist:
                min_dist = dist
                min_path = path
        sorted_waypoints = PoseArray()

        for idx in min_path:
            sorted_waypoints.poses.append(way_points.poses[idx])

        return sorted_waypoints

    def callback_waypoint(self, msg: PoseArray):
        self.way_points = PoseArray()
        for i, p in enumerate(msg.poses):
            self.way_points.poses.append(gps_pose_to_pose(p))

    def compute(self, task_score, task_timeout, task_state):
        if self.way_points is None:
            print('No WayPoints')
            return
        if self.pose is None:
            print('No Pose')
            return
        if task_state != 'running':
            return
        if self.best_way is None:
            self.best_way = self.find_way(self.way_points, self.pose)
            print(self.best_way)
        self.waypoint_pub.publish(self.best_way)


class StationKeeping():
    def __init__(self, node: MissionPlanner) -> None:
        self.goal_sub = node.create_subscription(
            PoseStamped, '/vrx/stationkeeping/goal', self.callback_goal, 10)
        self.waypoint_pub = node.create_publisher(
            PoseArray, '/wamv/waypoints', 10)
        self.goal = None
    def callback_goal(self, msg: PoseStamped):
        self.goal = gps_pose_to_pose(datum=DATUM, gps=msg.pose)

    def compute(self, task_score, task_timeout, task_state):
        if self.goal is not None:
            waypoints = PoseArray()
            waypoints.poses.append(self.goal)
            self.waypoint_pub.publish(waypoints)


def main(args=None):
    rclpy.init(args=args)
    planner = MissionPlanner()
    rclpy.spin(planner)
    planner.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
