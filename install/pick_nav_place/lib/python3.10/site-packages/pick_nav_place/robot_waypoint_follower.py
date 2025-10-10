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
    POSITION_LENGTH : int = 3
    WAYPOINT_POSITIONS : dict[str, tuple[float, float, float]] = {
        "home": (0.0, 0.0, 0.0),
        "conveyor": (2.7, -0.65, 0.0),
        "drop-off": (31.0, -4.25, 4.712),
    }

    def __init__(self):
        """

        """
        super().__init__('robot_waypoint_follower')

        self.client : ActionClient = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.waypoint_name : str = "Unknown"
        self._waypoint : tuple[float, float, float] = self.WAYPOINT_POSITIONS.get("home", (0.0, 0.0, 0.0))
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


    def _get_waypoint(self, key : str) -> bool:
        """
        [PROTECTED HELPER] Retrieves a waypoint position from the dictionary and updates the internal '_waypoint' attribute.

        Args:
            key (str): key (name) of the joint position to load.

        Returns:
            bool: True if the key exists and was loaded, False otherwise.
        """
        if key not in self.WAYPOINT_POSITIONS:
            self.get_logger().error(f"Unknown Waypoint Name: {key}")
            return False
        
        self._waypoint = self.WAYPOINT_POSITIONS[key]

        return True
    

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
    def go_to_waypoint(self, pos : str) -> bool:
        """
        Navigates the robot to specified waypoint position.

        Args:
            pos (str): string of chosen pre-defined waypoint position in WAYPOINT_POSITIONS (e.g., 'home', 'conveyor').

        Returns:
            bool: True if navigation was successful, False otherwise.
        """
        pos = pos.lower()
        if not self._get_waypoint(pos):
            # _get_waypoint handles error logging
            return False
        
        self.waypoint_name : str = pos

        self.get_logger().info(f"Navigating to {pos.upper()} position...")

        wp : PoseStamped = self._make_pose(*self._waypoint)

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
    

    def wait_at_waypoint(self, wait_seconds : float | None = None) -> None:
        """
        Pauses execution for a specified duration at the current location.

        Returns:
            None: sleep only method.
        """
        if not wait_seconds:
            wait_seconds = self.wait_seconds

        self.get_logger().info(f'Waiting {wait_seconds:.1f} seconds at waypoint {self.waypoint_name}...')
        time.sleep(wait_seconds)


    # --- Getters ---
    def get_waypoint_names(self) -> list[str]:
        """
        Retrieves the list of available joint position names (keys) stored in JOINT_POSITIONS.

        Returns:
            list[str]: list of all predefined position names.
        """
        return list(self.WAYPOINT_POSITIONS.keys())
    

    def get_current_waypoint_name(self) -> str:
        """
        Returns the name of the last successfully reached waypoint.

        Returns:
            str: name of the waypoint.
        """
        return self.waypoint_name
    

    # --- Setters ---
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


    def set_waypoint_position(self, new_waypoint : tuple[float, float, float], waypoint_name : str) -> bool:
        """
        Adds or updates a named joint position in the JOINT_POSITIONS dictionary.

        Args:
            new_waypoint (tuple[float, float, float]):   tuple of 3 float values representing the target joint angles (in radians).
            position_name (str):        unique or pre-existing name to assign or update this joint position.

        Returns:
            bool: True if position was successfully added/updated, False otherwise.
        """
        waypoint_name = waypoint_name.lower()

        # new position validation checks
        if len(new_waypoint) != self.POSITION_LENGTH:
            self.get_logger().error(f"Waypoint tuple size invalid. Expected {self.POSITION_LENGTH} values (x, y, yaw) but got {len(new_waypoint)}.")
            return False
        
        if not all(isinstance(val, (int, float)) for val in new_waypoint):
            self.get_logger().error("Waypoint values must be floats or integers.")
            return False
        
        if not isinstance(waypoint_name, str) or not waypoint_name.strip():
            self.get_logger().error("Waypoint name must be a non-empty string.")
            return False
        
        # set new joint position
        self.WAYPOINT_POSITIONS[waypoint_name] = tuple(float(val) for val in new_waypoint)
        self.get_logger().info(f"Updated/Added new waypoint position: '{waypoint_name}'!")

        return True



def main(args=None):
    """
    Initializes the ROS 2 node and spins the RobotWaypointFollower.
    """
    rclpy.init(args=args)
    
    node = RobotWaypointFollower()
    
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