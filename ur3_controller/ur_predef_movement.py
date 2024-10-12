#!/usr/bin/env python3

import os
import sys
import time
import yaml

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from ament_index_python.packages import get_package_share_directory  # Import added
from builtin_interfaces.msg import Duration
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.action import FollowJointTrajectory


class URPredefinedMovement(Node):

    def __init__(self):
        super().__init__("ur_predefined_movement")
        self.declare_parameter("controller_name", "scaled_joint_trajectory_controller")
        self.declare_parameter("joints", [
            "shoulder_pan_joint",
            "shoulder_lift_joint",
            "elbow_joint",
            "wrist_1_joint",
            "wrist_2_joint",
            "wrist_3_joint",
        ])

        controller_name = self.get_parameter("controller_name").value + "/follow_joint_trajectory"
        self.joints = self.get_parameter("joints").value

        if not self.joints:
            self.get_logger().error('"joints" parameter is required and cannot be empty.')
            raise ValueError('"joints" parameter is required.')

        # Determine the path to the trajectories.yaml file using get_package_share_directory
        package_share = get_package_share_directory('ur3_controller')
        trajectories_file = os.path.join(package_share, 'config', 'trajectories.yaml')

        self.get_logger().info(f"Loading trajectories from: {trajectories_file}")
        if not os.path.exists(trajectories_file):
            self.get_logger().error(f"Trajectories file does not exist: {trajectories_file}")
            raise FileNotFoundError(f"Trajectories file does not exist: {trajectories_file}")

        self._action_client = ActionClient(self, FollowJointTrajectory, controller_name)
        self.get_logger().info(f"Waiting for action server on {controller_name}")
        self._action_client.wait_for_server()

        self.load_trajectories(trajectories_file)
        self.current_traj_index = 0
        self._send_goal_future = None
        self.execute_next_trajectory()

    def load_trajectories(self, file_path):
        try:
            with open(file_path, 'r') as file:
                trajectories_data = yaml.safe_load(file)
        except FileNotFoundError:
            self.get_logger().error(f"Trajectories file not found: {file_path}")
            raise
        except yaml.YAMLError as e:
            self.get_logger().error(f"Error parsing YAML file: {e}")
            raise

        self.goals = {}
        for traj_name, points in trajectories_data.items():
            goal = JointTrajectory()
            goal.joint_names = self.joints
            for idx, pt in enumerate(points):
                point = JointTrajectoryPoint()

                # Validate positions
                if "positions" not in pt:
                    self.get_logger().error(f"Missing 'positions' in trajectory '{traj_name}' point {idx}.")
                    raise KeyError(f"Missing 'positions' in trajectory '{traj_name}' point {idx}.")

                point.positions = pt["positions"]

                # Handle velocities with default
                velocities = pt.get("velocities", [0.0] * len(self.joints))

                # Validate velocities
                if not isinstance(velocities, list):
                    self.get_logger().error(f"'velocities' must be a list in trajectory '{traj_name}' point {idx}.")
                    raise TypeError(f"'velocities' must be a list in trajectory '{traj_name}' point {idx}.")

                if len(velocities) != len(self.joints):
                    self.get_logger().error(f"Length of 'velocities' does not match number of joints in trajectory '{traj_name}' point {idx}.")
                    raise ValueError(f"Length of 'velocities' does not match number of joints in trajectory '{traj_name}' point {idx}.")

                # Ensure all velocities are floats
                try:
                    velocities = [float(v) for v in velocities]
                except ValueError:
                    self.get_logger().error(f"All 'velocities' must be floats in trajectory '{traj_name}' point {idx}.")
                    raise

                point.velocities = velocities

                # Handle time_from_start
                if "time_from_start" not in pt:
                    self.get_logger().error(f"Missing 'time_from_start' in trajectory '{traj_name}' point {idx}.")
                    raise KeyError(f"Missing 'time_from_start' in trajectory '{traj_name}' point {idx}.")

                time_from_start = pt["time_from_start"]
                point.time_from_start = Duration(
                    sec=time_from_start.get("sec", 0),
                    nanosec=time_from_start.get("nanosec", 0)
                )
                goal.points.append(point)
            self.goals[traj_name] = goal
        self.trajectory_names = list(self.goals.keys())

    def execute_next_trajectory(self):
        if self.current_traj_index >= len(self.trajectory_names):
            self.get_logger().info("All trajectories have been executed.")
            rclpy.shutdown()
            return

        traj_name = self.trajectory_names[self.current_traj_index]
        self.current_traj_index += 1
        self.execute_trajectory(traj_name)

    def execute_trajectory(self, traj_name):
        self.get_logger().info(f"Executing trajectory: {traj_name}")
        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory = self.goals[traj_name]
        goal_msg.goal_time_tolerance = Duration(sec=0, nanosec=1000000)

        self._send_goal_future = self._action_client.send_goal_async(goal_msg)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Goal rejected.")
            return

        self.get_logger().info("Goal accepted.")
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        status = self.error_code_to_str(result.error_code)
        self.get_logger().info(f"Trajectory execution result: {status}")

        if result.error_code == FollowJointTrajectory.Result.SUCCESSFUL:
            time.sleep(1)  # Optional: wait before next trajectory
            self.execute_next_trajectory()
        else:
            self.get_logger().error("Trajectory execution failed.")
            rclpy.shutdown()

    @staticmethod
    def error_code_to_str(error_code):
        mapping = {
            FollowJointTrajectory.Result.SUCCESSFUL: "SUCCESSFUL",
            FollowJointTrajectory.Result.INVALID_GOAL: "INVALID_GOAL",
            FollowJointTrajectory.Result.INVALID_JOINTS: "INVALID_JOINTS",
            FollowJointTrajectory.Result.OLD_HEADER_TIMESTAMP: "OLD_HEADER_TIMESTAMP",
            FollowJointTrajectory.Result.PATH_TOLERANCE_VIOLATED: "PATH_TOLERANCE_VIOLATED",
            FollowJointTrajectory.Result.GOAL_TOLERANCE_VIOLATED: "GOAL_TOLERANCE_VIOLATED",
        }
        return mapping.get(error_code, "UNKNOWN_ERROR")


def main(args=None):
    rclpy.init(args=args)
    try:
        movement_client = URPredefinedMovement()
        rclpy.spin(movement_client)
    except KeyboardInterrupt:
        movement_client.get_logger().info("Shutdown requested by user.")
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()