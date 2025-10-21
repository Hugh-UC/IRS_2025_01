#!/usr/bin/env python3
import math
import time
import json
import threading

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose, Spin
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
        "home": (0.0, 0.0, 0.1),
        "conveyor": (2.50, -0.65, 0.1),
        "conveyor_leave": (2.50, -0.65, 1.4),
        "drop-off": (31.0, -4.25, 4.712),
    }
    STATUS_LABELS : list[str] = ["idle", "busy", "complete", "error"]

    def __init__(self):
        """

        """
        super().__init__('robot_waypoint_follower')

        self._client : ActionClient         = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self._spin_client : ActionClient    = ActionClient(self, Spin, 'spin')

        self._waypoint_name : str = "Unknown"
        self._waypoint : tuple[float, float, float] = self.WAYPOINT_POSITIONS.get("home", (0.0, 0.0, 0.0))
        self._wait_seconds : float = 0.5

        
        self._status : int = 0

        self._goal_checker_pub = self.create_publisher(         # initialise ros2 topic publisher
            String, 
            '/pnp/current_goal_checker',        # <-- use the topic name from custom BT XML
            1
        )

        self._waypoint_status_pub = self.create_publisher(      # initialise waypoint status publisher
            String, 
            '/pnp/waypoint_follow_status',  # dedicated topic name
            10
        )

        # ROS2 Topic Subscriptions
        self._coordinator_sub = self.create_subscription(
            String,
            '/pnp/coordinator',
            self._listener_callback,
            10
        )

        self.get_logger().info('Waypoint Follower node started. Waiting for Nav2 action server...')

        nav_ready = self._client.wait_for_server(timeout_sec=5.0)
        spin_ready = self._spin_client.wait_for_server(timeout_sec=5.0)

        if not nav_ready or not spin_ready:
            self.get_logger().error('Nav2 action server not available after 5s.')
            self._status = 3
        else:
            self.get_logger().info('Nav2 action server ready.')


        self._publish_status()

        # locks thread to ensure only one goal is being processed at a time
        self._goal_lock = threading.Lock()



    # --- Listener Method ---
    def _listener_callback(self, msg : String) -> None:
        """
        Parses the JSON command, checks the target, and executes the specified method.
        """
        try:
            data = json.loads(msg.data)
            
            # 1. Check if the command is for this node
            target = data.get("node")
            if target != self.get_name():
                return
            
            command_data = data.get("command", {})
            method_name = command_data.get("method")
            value = command_data.get("value")

            if not method_name:
                self.get_logger().warn("Received command without a specified method.")
                return

            # 2. Check if the method exists and is callable
            method = getattr(self, method_name, None)
            
            if method is None or not callable(method):
                self.get_logger().error(f"Command Error: Method '{method_name}' not found or not callable.")
                self._status = 3
                self._publish_status()
                return

            # 3. Execute the method
            self.get_logger().info(f"Executing command: {method_name}({value})")
            
            self._execute_new_thread(method, value)

        except Exception as e:
            self.get_logger().error(f"Failed to process coordinator command: {e}\nRaw msg={msg.data}")
            self._status = 3
            self._publish_status()


    def _execute_new_thread(self, method, value = None) -> None:
        try:
            thread_args : tuple = ()

            if value is not None and value != "":
                thread_args = (value,)

            t = threading.Thread(target=method, args=thread_args, daemon=True)
            t.start()
            self.get_logger().info(f"New execution thread started.")

        except Exception as e:
            self.get_logger().error(f"Error during threaded execution of {method.__name__}: {e}")
            self._status = 3
            self._publish_status()


    # --- Publisher Methods --
    def _select_goal_checker(self, precise_check = False):
        msg = String()
        if precise_check:
            msg.data = "precise_goal_checker"       # use higher precision goal checker
            self.get_logger().info('Publishing PRECISE Goal Checker ID.')
        else:
            msg.data = "general_goal_checker"
            self.get_logger().info('Publishing GENERAL Goal Checker ID.')

        self._goal_checker_pub.publish(msg)
        time.sleep(0.1)         # give a moment for message to be processed by GoalCheckerSelector


    def _publish_status(self) -> None:
        # create message payload
        payload : dict[str, str | int] = {
            "node": self.get_name(),                # "robot_waypoint_follower"
            "current_target": self._waypoint_name,
            "status": self._get_status_name(),       # "idle", "busy", "complete", "error"
            "timestamp": int(self.get_clock().now().nanoseconds / 1e9), # Unix timestamp
        }

        json_payload : json = json.dumps(payload)

        msg : String = String()
        msg.data = json_payload

        self._waypoint_status_pub.publish(msg)



    # --- Method to send a goal and wait for result ---
    def _send_and_wait(self, pose : PoseStamped, precise_check = False) -> bool:
        """
        Sends a single Nav2 goal and blocks until the robot reaches the goal or fails.

        Args:
            pose (PoseStamped): target pose for the robot.

        Returns:
            bool: True if goal was reached, False otherwise.
        """
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
        send_future = self._client.send_goal_async(goal, feedback_callback=feedback_cb)

        # rclpy.spin_until_future_complete(self, send_future)
        while not send_future.done():
            time.sleep(0.05)

        handle = send_future.result()

        if not handle or not handle.accepted:
            self.get_logger().error('Goal was rejected!')
            self._status = 3
            self._publish_status()
            return False

        # 5. Wait until navigation is complete
        result_future = handle.get_result_async()

        # rclpy.spin_until_future_complete(self, result_future)
        while not result_future.done():
            time.sleep(0.05)

        result = result_future.result()

        # 6. Log and Return Result
        if result is None or result.status != 4:
            self.get_logger().error('Goal failed or was interupted.')
            self._status = 3
            self._publish_status()
            return False

        self.get_logger().info('Goal reached successfully!')
        self._status = 2
        self._publish_status()
        
        return True


    def _send_spin_and_wait(self, yaw_rad = 0.0, precise_check = False) -> bool:
        # 1. Prepare Goal
        goal : Spin.Goal = Spin.Goal()
        goal.target_yaw = float(yaw_rad)

        self.get_logger().info(f"Sending Spin goal for {yaw_rad:.2f} radians...")

        # 2. Select the Goal Checker
        self._select_goal_checker(precise_check)

        # 3. Send and Wait for Goal Acceptance
        send_future = self._spin_client.send_goal_async(goal)

        while not send_future.done():
            time.sleep(0.05)
        
        handle = send_future.result()

        handle = send_future.result()

        if not handle or not handle.accepted:
            self.get_logger().error('Goal was rejected!')
            self._status = 3
            self._publish_status()
            return False

        # 5. Wait until navigation is complete
        result_future = handle.get_result_async()

        # rclpy.spin_until_future_complete(self, result_future)
        while not result_future.done():
            time.sleep(0.05)

        result = result_future.result()

        # 6. Log and Return Result
        if result is None or result.status != 4:
            self.get_logger().error('No result returned.')
            self._status = 3
            self._publish_status()
            return False

        self.get_logger().info('Goal reached successfully!')
        self._status = 2
        self._publish_status()
        
        return True


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


    # --- Waypoint Methods ---
    def _go_to_waypoint(self, pos : str) -> bool:
        """
        Navigates the robot to specified waypoint position.

        Args:
            pos (str): string of chosen pre-defined waypoint position in WAYPOINT_POSITIONS (e.g., 'home', 'conveyor').

        Returns:
            bool: True if navigation was successful, False otherwise.
        """
        with self._goal_lock:
            if not self._check_status():
                # _check_status handles error logging
                return False
            
            pos = pos.lower()
            if not self._get_waypoint(pos):
                # _get_waypoint handles error logging
                return False
            
            self._waypoint_name : str = pos

            self._status = 1
            self._publish_status()

            self.get_logger().info(f"Navigating to {pos.upper()} position...")

            wp : PoseStamped = self._make_pose(*self._waypoint)

            result : bool = self._send_and_wait(wp)

            self._wait_at_waypoint()
            self._status = 0
            self._publish_status()

            return result


    def _wait_at_waypoint(self, wait_seconds : float | None = None) -> None:
        """
        Pauses execution for a specified duration at the current location.

        Returns:
            None: sleep only method.
        """
        if not wait_seconds:
            wait_seconds = self._wait_seconds

        self.get_logger().info(f'Waiting {wait_seconds:.1f} seconds at waypoint {self._waypoint_name}...')
        time.sleep(wait_seconds)


    def _check_status(self) -> bool:
        """
        _summary_

        Returns:
            bool: _description_
        """
        if self._status == 1:
            self.get_logger().info("Robot Navigation is 'busy'. Ignoring new command(s).")
            return False
        
        if self._status == 3:
            self.get_logger().error("Error with Robot Navigation. Ignoring new command(s).")
            return False
        
        return True


    # --- Spin Methods ---
    def _spin(self, yaw_rad : float) -> bool:
        with self._goal_lock:
            if not self._check_status():
                # _check_status handles error logging
                return False
            
            self._action_name = "SPINNING (Action)"
            self._status = 1
            self._publish_status()

            result : bool = self._send_spin_and_wait(yaw_rad)

            self._wait_at_waypoint()
            self._status = 0
            self._publish_status()

            return result



    # --- Getters ---
    def _get_waypoint_names(self) -> list[str]:
        """
        Retrieves the list of available joint position names (keys) stored in JOINT_POSITIONS.

        Returns:
            list[str]: list of all predefined position names.
        """
        return list(self.WAYPOINT_POSITIONS.keys())
    

    def _get_current_waypoint_name(self) -> str:
        """
        Returns the name of the last successfully reached waypoint.

        Returns:
            str: name of the waypoint.
        """
        return self._waypoint_name


    def _get_status_name(self) -> str:
        """
        Takes the current status of the robot arm and returns its translated string representation.

        Returns:
            str: string of status name
        """
        return self.STATUS_LABELS[self._status]


    # --- Setters ---
    def _set_wait_seconds(self, seconds : float) -> bool:
        """
        Sets the duration (in seconds) that the robot waits after reaching a waypoint.

        Args:
            seconds (float): new wait time in seconds.

        Returns:
            bool: True if value is updated, False otherwise.
        """
        try:
            self._wait_seconds : float = float(seconds)
        except ValueError:
            return False
        
        return True


    def _set_waypoint_position(self, new_waypoint : tuple[float, float, float], waypoint_name : str) -> bool:
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

    executor = MultiThreadedExecutor()
    executor.add_node(node)
    
    try:
        # Spin the node to keep the ActionClient alive and ready for calls
        node.get_logger().info('RobotWaypointFollower is now spinning and ready to accept commands!')
        executor.spin()
    except KeyboardInterrupt:
        node.get_logger().info('Navigation Commander stopped by user.')
    finally:
        node.get_logger().info('Navigation sequence complete. Shutting down.')
        executor.remove_node(node)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()