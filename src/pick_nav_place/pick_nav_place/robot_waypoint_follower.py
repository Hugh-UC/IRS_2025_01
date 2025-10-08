#!/usr/bin/env python3
import math
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from std_msgs.msg import String

from typing import Callable



class RobotWaypointFollower(Node):
    """
    ROS 2 Node that acts as a navigation service. 
    It communicates with the Nav2 stack via the NavigateToPose Action Client 
    and exposes methods for high-level waypoint navigation.

    Args:
        Node (rclpy.node): node class from ROS 2 client library,
                           core node functionality is inherited from class.
    """
    def __init__(self):
        """

        """
        super().__init__('robot_waypoint_follower')

        self.client : ActionClient = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.waypoint_name : str = "Unknown"
        self.wait_seconds : float = 0.5

        # initialise ros2 topic publisher
        self.goal_checker_pub = self.create_publisher(
            String, 
            '/pnp/current_goal_checker',        # <-- use the topic name from custom BT XML
            1
        )

        # initialise waypoint status publisher
        self.waypoint_status_pub = self.create_publisher(
            String, 
            '/pnp/waypoint_follow_status',  # dedicated topic name
            10
        )

        self.get_logger().info('Waypoint Follower node started. Waiting for Nav2 action server...')


    # --- Method to send a goal and wait for result ---
    def _send_and_wait(self, pose : PoseStamped, precise_check = False) -> bool:
        """
        Sends a single Nav2 goal and blocks until the robot reaches the goal or fails.

        Args:
            pose (PoseStamped): target pose for the robot.

        Returns:
            bool: True if goal was reached, False otherwise.
        """
        if not self.client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('Nav2 action server not available after 5s.')
            return False
        else:
            self.get_logger().info('Nav2 action server ready.')
        
        # 1. Prepare Goal
        pose.header.stamp = self.get_clock().now().to_msg()
        goal : NavigateToPose.Goal = NavigateToPose.Goal()
        goal.pose = pose

        # 2. Select the Goal Checker
        self._select_goal_checker(precise_check)

        # 3. Define Feedback Callback
        def feedback_cb(fb):
            try:
                dist : float = fb.feedback.distance_remaining
                self.get_logger().info(f'Distance remaining: {dist:.2f} m', throttle_duration_sec=1.0)
            except Exception:
                pass
        
        # 4. Send and Wait for Goal Acceptance
        send_future = self.client.send_goal_async(goal, feedback_callback=feedback_cb)
        rclpy.spin_until_future_complete(self, send_future)
        handle = send_future.result()

        if not handle or not handle.accepted:
            self.get_logger().error('Goal was rejected!')
            return False

        # 5. Wait until navigation is complete
        result_future = handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        result = result_future.result()

        # 6. Log and Return Result
        if result is None:
            self.get_logger().error('No result returned.')
            return False

        self.get_logger().info('Goal reached successfully!')
        return True
    

    def _select_goal_checker(self, precise_check = False):
        msg = String()
        if precise_check:
            msg.data = "precise_goal_checker"       # use higher precision goal checker
            self.get_logger().info('Publishing PRECISE Goal Checker ID.')
        else:
            msg.data = "general_goal_checker"
            self.get_logger().info('Publishing GENERAL Goal Checker ID.')

        self.goal_checker_pub.publish(msg)
        time.sleep(0.1)         # give a moment for message to be processed by GoalCheckerSelector


    # --- Helper method to build a PoseStamped ---
    def _make_pose(self, x : float, y : float, yaw : float) -> PoseStamped:
        """
        Create a PoseStamped (position + orientation) in the 'map' frame.

        Args:
            x (float):      x-coordinate in meters
            y (float):      y-coordinate in meters
            yaw (float):    robot orientation (heading) in radians

        Returns:
            PoseStamped: PoseStamped message ready for Nav2.
        """
        ps : PoseStamped = PoseStamped()
        ps.header.frame_id = 'map'  # always use 'map' for navigation goals
        ps.pose.position.x = x
        ps.pose.position.y = y

        # Convert yaw (in radians) into quaternion (needed by ROS2)
        half : float = yaw * 0.5
        ps.pose.orientation.z = math.sin(half)
        ps.pose.orientation.w = math.cos(half)
        return ps


    # --- Waypoint methods ---
    def go_home(self) -> bool:
        """
        Navigates the robot to Home position (0.0, 0.0, 0.0).

        Returns:
            bool: True if navigation was successful, False otherwise.
        """
        self.get_logger().info("Navigating to Home Position (0, 0, 0)")
        self.waypoint_name : str = "Home"

        wp : PoseStamped = self._make_pose(0.0, 0.0, 0.0)

        return self._send_and_wait(wp)
    

    def go_to_conveyor(self) -> bool:
        """
        Navigates the robot to Conveyor pickup position ().

        Returns:
            bool: True if navigation was successful, False otherwise.
        """
        self.get_logger().info("Navigating to Conveyor Position")
        self.waypoint_name : str = "Conveyor"

        wp : PoseStamped = self._make_pose(2.7, -0.596, 0.0)
        
        return self._send_and_wait(wp)


    def go_to_dropoff(self) -> bool:
        """
        Navigates to the Dropoff Station using a multiple-waypoint sequence.

        Returns:
            bool: True if navigation was successful, False otherwise.
        """
        self.get_logger().info("Navigating to Dropoff Station")
        wp : list[PoseStamped] = [
            self._make_pose(0.0, 0.0, 0.0),
            self._make_pose(5.81, 14.0, 0.0),
            self._make_pose(24.0, 13.0, 4.712),
            self._make_pose(28.5, 4.33, 4.0),
            self._make_pose(28.3, -3.27, 0.0),
            self._make_pose(6.23, 0.035, 3.14)
        ]

        return self._run_multiple_poses(wp)
        
    
    def _run_multiple_poses(self, poses : list[PoseStamped]) -> bool:
        """
        Executes a sequence of navigation goals one after the other.

        Args:
            poses (list[PoseStamped]): list of PoseStamped messages representing the route.

        Returns:
            bool: True if all poses were successfully reached, False otherwise.
        """
        self.get_logger().info(f"Starting multi-pose sequence with {len(poses)} waypoints.")

        for i, pose in enumerate(poses):
            self.waypoint_name : str = f"({i + 1})"

            if not self._send_and_wait(pose):
                self.get_logger().error(f"Failed to reach waypoint ({i+1}). Aborting sequence.")
                return False
            
            # Pause after success
            self.wait_at_waypoint()
            
        return True
    

    def wait_at_waypoint(self) -> None:
        """
        Pauses execution for a specified duration at the current location.

        Returns:
            None: sleep only method.
        """
        self.get_logger().info(f'Waiting {self.wait_seconds:.1f} seconds at waypoint {self.waypoint_name}...')
        time.sleep(self.wait_seconds)


    # --- Getters and Setters ---
    def get_waypoint_name(self) -> str:
        """
        Returns the name of the last successfully reached waypoint.

        Returns:
            str: name of the waypoint.
        """
        return self.waypoint_name
    

    def set_wait_seconds(self, seconds : float) -> bool:
        """
        Sets the duration (in seconds) that the robot waits after reaching a waypoint.

        Args:
            seconds (float): new wait time in seconds.

        Returns:
            bool: True if value is updated, False otherwise.
        """
        try:
            self.wait_seconds : float = float(seconds)
        except ValueError:
            return False
        
        return True



def main(args=None):
    """
    Initializes the ROS 2 node and spins the RobotWaypointFollower.
    """
    rclpy.init(args=args)
    
    node = RobotWaypointFollower()

    # node.go_to_conveyor()
    
    try:
        # Spin the node to keep the ActionClient alive and ready for calls
        node.get_logger().info('RobotWaypointFollower is now spinning and ready to accept commands!')
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Navigation Commander stopped by user.')
    finally:
        node.get_logger().info('Navigation sequence complete. Shutting down.')
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()