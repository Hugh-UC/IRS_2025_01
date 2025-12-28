#!/usr/bin/env python3
"""
robot_waypoint_follower.py

Author:         Hugh Brennan
University:     University of Canberra

Project:        Pinapple Grand Challenge
Version:        2.0.0v
Created:        23/08/2025
Updated:        21/11/2025

Description:
    ROS 2 node for high-level waypoint navigation using Nav2.
    Provides action server for navigation and spin commands.
"""
import math
import time
import json
import threading

from typing import Callable

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.action import ActionClient, ActionServer, GoalResponse
from rclpy.action.server import ServerGoalHandle
from rclpy.action.client import ClientGoalHandle
from rclpy.task import Future
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose, Spin
from action_msgs.msg import GoalStatus
from std_msgs.msg import String

# --- Import for Custom Action ---
from pnp_interfaces.action import WaypointFollower
from pnp_interfaces.msg import CommandArguments, NavOptions, WaitDuration



class RobotWaypointFollower(Node):
    """
    ROS 2 Node that acts as a navigation service. 
    It communicates with the Nav2 stack via the NavigateToPose Action Client 
    and exposes methods for high-level waypoint navigation.

    Args:
        Node (rclpy.node): node class from ROS 2 client library,
                           core node functionality is inherited from class.
    """
    # --- Class Constants ---
    NODE_NAME : str             = 'robot_waypoint_follower'
    ACTION_NAMES : list[str]    = ['navigate_to_pose', 'spin']
    POSITION_LENGTH : int       = 3
    WAYPOINT_POSITIONS : dict[str, tuple[float, float, float]] = {
        "home": (0.0, 0.0, 0.1),
        "conveyor": (2.50, -0.65, 0.1),
        "conveyor_leave": (2.50, -0.65, 1.4),
        "drop-off": (30.6, -4.50, 4.712),
    }
    STATE_LABELS : list[str] = ["idle", "busy", "complete", "error"]
    SERVICE_CALL_TIMEOUT : float = 5.0
    EXECUTION_TRHOTTLE : float = 0.05

    def __init__(self) -> None:
        """
        Initializes the node, Nav2 Action Clients, and the WaypointFollower Action Server.
        """
        super().__init__(self.NODE_NAME)

        self._client : ActionClient         = ActionClient(self, NavigateToPose, self.ACTION_NAMES[0])
        self._spin_client : ActionClient    = ActionClient(self, Spin, self.ACTION_NAMES[1])
        self._server : ActionServer         = ActionServer(    # Create ROS 2 Action Server
            self,
            WaypointFollower,
            f'/pnp/{self.NODE_NAME}',
            self._execute_callback,
            goal_callback=self._goal_callback
        )

        self._waypoint : tuple[float, float, float] = self.WAYPOINT_POSITIONS.get("home", (0.0, 0.0, 0.0))
        self._waypoint_name : str   = "Unknown"     # current robot position
        self._spin_yaw : float      = 0.0           # target spin distance (radians)
        self._wait_seconds : float  = 0.5           # target wait seconds
        self._state : int = 0      # 0: idle, 1: busy, 2: complete, 3: error

        # store current goal handle
        self._goal_handle : ServerGoalHandle | None = None

        # ROS 2 Topic Publishers
        self._goal_checker_qos : QoSProfile = QoSProfile(
            depth = 1,
            reliability = ReliabilityPolicy.RELIABLE,
            durability = DurabilityPolicy.TRANSIENT_LOCAL
        )

        self._goal_checker_pub = self.create_publisher(     # initialise ros2 topic publisher
            String, 
            '/pnp/current_goal_checker',                    # topic name from custom BT XML
            self._goal_checker_qos                          # QOS profile for BT Node communication
        )

        self.get_logger().info('Waypoint Follower node started. Waiting for Nav2 action server...')

        nav_ready : bool = self._client.wait_for_server(timeout_sec=self.SERVICE_CALL_TIMEOUT)
        spin_ready : bool = self._spin_client.wait_for_server(timeout_sec=self.SERVICE_CALL_TIMEOUT)

        if not nav_ready or not spin_ready:
            self.get_logger().error('Nav2 action server not available after 5s.')
            self._state = 3
        else:
            self.get_logger().info('Nav2 action server ready.')

        # locks thread to ensure only one goal is being processed at a time
        self._goal_lock = threading.Lock()


    # --- Action Server Callbacks ---s
    def _goal_callback(self, goal_request : WaypointFollower.Goal) -> GoalResponse:
        """
        Accepts or rejects a new goal based on command validity and node status.

        Args:
            goal_request (WaypointFollower.Goal): incoming goal request.

        Returns:
            GoalResponse: ACCEPT if the command is valid and node is ready, else REJECT.
        """
        self.get_logger().info(f'Received Nav Action Goal: {goal_request.command}')

        # check if command is valid class method
        method : Callable | None = getattr(self, goal_request.command, None)
        if not method or not callable(method):
            self.get_logger().warn("Command Failed: Unknown command received. Rejecting new goal.")
            return GoalResponse.REJECT

        # check state ('busy'/'error')
        if not self._check_state():
            # _check_state handles error logging
            return GoalResponse.REJECT
        
        self.get_logger().info("Command Accepted: New nav goal received!")
        return GoalResponse.ACCEPT


    def _execute_callback(self, goal_handle : ServerGoalHandle) -> WaypointFollower.Result:
        """
        Called when the goal is accepted.
        Runs accepted goal and returns the final result.

        Args:
            goal_handle (ServerGoalHandle): handle for the accepted goal.

        Returns:
            WaypointFollower.Result: result of the action.
        """
        # update class goal handle
        self._goal_handle = goal_handle

        # update internal state to busy
        self._state = 1
        self._publish_feedback("Executing Nav goal...", log_type="info")

        # execute goal
        result : WaypointFollower.Result = self._run_goal(goal_handle)

        return result
    

    # --- Goal Handle ---
    def _run_goal(self, goal_handle : ServerGoalHandle) -> WaypointFollower.Result:
        """
        Performs the synchronous/blocking work for the Action Goal.
        Parses the CommandArguments message to retrieve goal.

        Args:
            goal_handle (ServerGoalHandle): handle for the active action goal.

        Returns:
            WaypointFollower.Result: result of the action.
        """
        request = goal_handle.request
        result = WaypointFollower.Result()

        success : bool = False

        with self._goal_lock:
            try:
                # retrieve command (method) and arguments
                command : str           = request.command
                args : CommandArguments = request.command_arguments

                # get callable command method (validated by '_goal_callback()')
                method : Callable = getattr(self, command)

                # execute the method
                success = method(args)

            except Exception as e:
                result.msg = f"Exception Thrown: {e}"
                self.get_logger().error(result.msg)
                success = False

            # final result
            result.success = success
            
            if success:
                result.msg = "Success: Nav goal completed."
                goal_handle.succeed()
                self._state = 0        # update internal status 'idle'

            else:
                if not result.msg:
                    result.msg = "Unknown Error: Nav goal failed."
                goal_handle.abort()
                self._state = 3        # update internal status 'error'
            
            # publish final status
            self._publish_feedback(f"Goal {command} finished.", log_type="info")
            self._goal_handle = None    # clear goal handle

        return result


    def _publish_feedback(self, msg : str = "", dist : float | None = None, log_type : str = "") -> None:
        """
        Publishes feedback to the active goal handle.

        Args:
            msg (str, optional): main feedback message. Defaults to "".
            dist (float | None, optional): distance remaining to goal. Defaults to None.
            log_type (str, optional): log message type ("info", "warn", "error", "fatal"). Defaults to "".
        """
        # exit if goal handle is not set
        if not self._goal_handle:
            self.get_logger().warn("Cannot publish feedback, no active goal handle.")
            return
        
        # log message (using defined type)
        match log_type:
            case "info":
                self.get_logger().info(msg)
            case "warn":
                self.get_logger().warn(msg)
            case "error":
                self.get_logger().error(msg)
            case "fatal":
                self.get_logger().fatal(msg)
            case _:
                pass

        # create new feedback message
        feedback : WaypointFollower.Feedback    = WaypointFollower.Feedback()
        feedback.current_status                 = self._get_state_name()
        feedback.status_description             = msg

        if dist is not None:
            feedback.distance_remaining = float(dist)

        # publish feedback message
        self._goal_handle.publish_feedback(feedback)


    def _get_state_name(self) -> str:
        """
        Takes the current status of the robot arm and returns its translated string representation.

        Returns:
            str: string of status name
        """
        return self.STATE_LABELS[self._state]


    def _check_state(self) -> bool:
        """
        Checks if the node is in a state to accept new goals.

        Returns:
            bool: True if Node is ready for a new command, False otherwise
        """
        if self._state == 1:
            self.get_logger().warn("Command Failed: Robot Navigation is 'busy'. Rejecting new goal.")
            return False
        
        if self._state == 3:
            self.get_logger().error("Command Failed: Error with Robot Navigation. Ignoring new command(s).")
            return False
        
        return True

    # _clear_error is currently depricated
    def _clear_error(self) -> None:
        """
        Attempts to clear an error state by re-checking Nav2 server readiness.
        """
        self.get_logger().warn('Attempting to clear error status by re-checking Nav2 server readiness...')

        nav_ready = self._client.wait_for_server(timeout_sec=self.SERVICE_CALL_TIMEOUT)
        spin_ready = self._spin_client.wait_for_server(timeout_sec=self.SERVICE_CALL_TIMEOUT)

        if nav_ready and spin_ready:
            self.get_logger().info('Nav2 action servers are READY. Resetting state to IDLE (0).')
            self._state = 0


    # --- Goal Checker Publisher ---
    def _select_goal_checker(self, precise_check = False) -> None:
        """
        Publishes a message to select the goal checker type for navigation.

        Args:
            precise_check (bool, optional): If True, selects the precise goal checker. Defaults to False.
        """
        msg = String()
        if precise_check:
            msg.data = "precise_goal_checker"       # use higher precision goal checker
            self._publish_feedback("Publishing PRECISE Goal Checker ID.", log_type="info")
        else:
            msg.data = "general_goal_checker"
            self._publish_feedback("Publishing GENERAL Goal Checker ID.", log_type="info")

        self._goal_checker_pub.publish(msg)
        time.sleep(self.EXECUTION_TRHOTTLE)         # give a moment for message to be processed by GoalCheckerSelector


    # --- Method to send a goal and wait for result ---
    def _send_and_wait(self, precise_check : bool = False) -> bool:
        """
        Sends a single Nav2 goal and blocks until the robot reaches the goal or fails.

        Args:
            pose (PoseStamped): target pose for the robot.
            precise_check (bool): True to use precise goal checker, False for general.

        Returns:
            bool: True if goal was reached, False otherwise.
        """
        # 1. Prepare Goal
        pose : PoseStamped          = self._make_pose(*self._waypoint)
        goal : NavigateToPose.Goal  = NavigateToPose.Goal()
        goal.pose                   = pose
        pose.header.stamp           = self.get_clock().now().to_msg()

        # 2. Select the Goal Checker
        self._select_goal_checker(precise_check)

        # 3. Define Feedback Callback
        def nav_feedback_cb(fb : NavigateToPose.FeedbackMessage) -> None:
            try:
                # retrieve distance remaining to waypoint (meters)
                dist : float        = fb.feedback.distance_remaining
                # publish feedback message
                feedback_msg : str  = f"Distance remaining: {dist:.2f} m"
                self.get_logger().info(feedback_msg, throttle_duration_sec=1.0)
                self._publish_feedback(feedback_msg, dist)
            except Exception as e:
                # publish feedback message
                feedback_msg : str = f"Exception Thrown: {e}"
                self.get_logger().info(feedback_msg, throttle_duration_sec=1.0)
                self._publish_feedback(feedback_msg)
        
        # 4. Send and Wait for Goal Response
        send_future : Future = self._client.send_goal_async(goal, feedback_callback=nav_feedback_cb)
        while not send_future.done():
            time.sleep(self.EXECUTION_TRHOTTLE)

        # 5. Check for Goal Acceptance
        handle : ClientGoalHandle | None = send_future.result()
        if not handle or not handle.accepted:
            self._state = 3
            # publish new feedback message
            self._publish_feedback("Nav2 Goal was rejected by server.", log_type="error")
            return False
        
        # 6. Publish Goal Accepted Feedback
        self._publish_feedback("Nav2 Goal accepted. Waiting for results...", log_type="info")

        # 7. Wait Until Navigation Goal Complete
        result_future : Future = handle.get_result_async()
        while not result_future.done():
            time.sleep(self.EXECUTION_TRHOTTLE)

        result : NavigateToPose.Result | None = result_future.result()

        # 8. Check for Successful Goal Status (GoalStatus.SUCCEEDED or 4); Log and Return Result
        #   depricated: result is None or result.status != 4
        success : bool = bool(result and result.status == GoalStatus.STATUS_SUCCEEDED)
        if success:
            self._publish_feedback("Nav Goal reached successfully.", log_type="info")
            self._state = 2
        else:
            self._publish_feedback(f"Goal failed with status: {result.status}", log_type="error")
            self._waypoint_name = "Unknown"
            self._state = 3
        
        return success


    def _send_spin_and_wait(self, precise_check : bool = False) -> bool:
        """
        Sends a spin goal to Nav2 and waits for completion.

        Args:
            precise_check (bool, optional): If True, selects the precise goal checker. Defaults to False.

        Returns:
            bool: True if goal was reached, False otherwise.
        """
        # 1. Prepare Goal
        goal : Spin.Goal    = Spin.Goal()
        goal.target_yaw     = float(self._spin_yaw)

        # 2. Select the Goal Checker
        self._select_goal_checker(precise_check)

        # 3. Define Feedback Callback
        def spin_feedback_cb(fb) -> None:
            try:
                # retrieve/calculate angle traveled/remaining in radians
                angle_traveled : float  = fb.feedback.angular_distance_traveled
                angle_remaining : float = self._spin_yaw - angle_traveled
                # publish feedback message
                feedback_msg : str = f"Spin Progress: {angle_traveled:.2f} rad(s), Remaining: {angle_remaining:.2f} rad(s)"
                self.get_logger().info(feedback_msg, throttle_duration_sec=1.0)
                self._publish_feedback(feedback_msg, angle_remaining)
            except Exception as e:
                # publish feedback message
                feedback_msg : str = f"Exception Thrown: {e}"
                self.get_logger().info(feedback_msg, throttle_duration_sec=1.0)
                self._publish_feedback(feedback_msg)

        # 4. Send and Wait for Goal Acceptance
        send_future : Future = self._spin_client.send_goal_async(goal, feedback_callback=spin_feedback_cb)
        while not send_future.done():
            time.sleep(self.EXECUTION_TRHOTTLE)
        
        # 5. Check for Goal Acceptance
        handle : ClientGoalHandle | None = send_future.result()
        if not handle or not handle.accepted:
            self._state = 3
            # publish new feedback message
            self._publish_feedback("Nav2 Spin Goal was rejected by server.", log_type="error")
            return False

        # 6. Publish Goal Accepted Feedback
        self._publish_feedback("Nav2 Spin Goal accepted. Waiting for results...", log_type="info")

        # 7. Wait until navigation is complete
        result_future : Future = handle.get_result_async()
        while not result_future.done():
            time.sleep(self.EXECUTION_TRHOTTLE)

        result : Spin.Result | None = result_future.result()

        # 8. Log and Return Result
        success : bool = bool(result and result.status == GoalStatus.STATUS_SUCCEEDED)
        if success:
            self._publish_feedback("Nav Spin Goal reached successfully.", log_type="info")
            self._state = 2
        else:
            self._publish_feedback(f"Spin Goal failed with status: {result.status}", log_type="error")
            self._waypoint_name = "Unknown"
            self._state = 3

        return success
    

    # --- Helper Method to Build a Nav2 Goal ---
    def _get_waypoint(self, key : str) -> bool:
        """
        Retrieves a waypoint position from the dictionary and updates the internal '_waypoint' attribute.

        Args:
            key (str): key (name) of the joint position to load.

        Returns:
            bool: True if the key exists and was loaded, False otherwise.
        """
        if key not in self.WAYPOINT_POSITIONS:
            self._state = 3
            # publish new feedback message
            self._publish_feedback(f"Unknown Waypoint Name: {key}", log_type="error")
            return False
        
        self._waypoint = self.WAYPOINT_POSITIONS[key]

        # publish new feedback message
        self._publish_feedback(f"Selected Waypoint ({key}): {self._waypoint}", log_type="info")

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


    # --- Waypoint Method ---
    def _go_to_waypoint(self, args : CommandArguments) -> bool:
        """
        Navigates the robot to specified waypoint position.

        Args:
            args (CommandArguments): command arguments containing chosen waypoint position from WAYPOINT_POSITIONS (e.g., 'home', 'conveyor').

        Returns:
            bool: True if navigation was successful, False otherwise.
        """
        # retrieve target waypoint
        pos : str = args.nav_goal.target_waypoint
        pos = pos.lower()
        if not self._get_waypoint(pos):
            # _get_waypoint handles error logging
            return False
        
        self._waypoint_name : str = pos

        # publish new feedback message
        self._publish_feedback(f"Navigating to {pos.upper()} position...", log_type="info")

        # send and retrieve goal result
        result : bool = self._send_and_wait(precise_check=True)

        # pause in current position
        self._wait_at_waypoint()

        return result


    # --- Spin Method ---
    def _spin(self, args : CommandArguments) -> bool:
        """
        Spins the robot by a specified yaw.

        Args:
            args (CommandArguments): command arguments containing target yaw rotation

        Returns:
            bool: True if spin was successful, False otherwise.
        """
        # retrieve target yaw angle (radians)
        yaw_rad : float = args.nav_goal.target_spin
        self._spin_yaw = yaw_rad

        # publish new feedback message
        self._publish_feedback(f"Sending Spin goal for {yaw_rad:.2f} radians...")

        # send and retrieve goal result
        result : bool = self._send_spin_and_wait(precise_check=True)

        # pause in current position
        self._wait_at_waypoint()

        return result


    # --- Wait Method ---
    def _wait_at_waypoint(self, args : CommandArguments | None = None) -> bool:
        """
        Pauses execution for a specified duration at the current location.

        Args:
            args (CommandArguments | None, optional): arguments containing WaitDuration. If None, uses class default. Defaults to None.

        Returns:
            bool: True after sleep complete.
        """
        # use class default if none passed
        if args is None:
            wait_seconds : float = self._wait_seconds
        else:
            wait_seconds : float = args.wait_goal.duration
            try:
                wait_seconds = float(wait_seconds)
            except ValueError as e:
                self.get_logger().error(f"ValueError: Could not use '{wait_seconds}' as duration for '_wait_at_position': {e}")
                return False

        # use class default
        if wait_seconds == -1.0:
            wait_seconds = self._wait_seconds

        # skip wait
        if wait_seconds <= 0.0:
            return False
        
        # publish new feedback message
        self._publish_feedback(f"Waiting {wait_seconds:.1f} seconds at waypoint {self._waypoint_name}...")

        # wait for defined duration
        time.sleep(wait_seconds)

        return True


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


    # --- Setters ---
    def _set_wait_seconds(self, args : CommandArguments = None) -> bool:
        """
        Sets the duration (in seconds) that the robot waits after reaching a waypoint.

        Args:
            args (CommandArguments): contains new wait time in seconds. Defaults to None.

        Returns:
            bool: True if value is updated, False otherwise.
        """
        if args is None or not isinstance(args, CommandArguments):
            # log and return error feedback
            return False
        
        try:
            # attempt to convert to float
            wait_seconds = float(args.wait_goal.duration)

            # ignore negative values
            if wait_seconds < 0.0:
                # publish new feedback message
                self._publish_feedback(f"Warning: Value must be > or = 0.0. Ignoring attempt to set wait duration '{wait_seconds}'", log_type="warn")
                return False
            
            # update duration
            self._wait_seconds = wait_seconds

            self._publish_feedback(f"Default wait duration set to {wait_seconds}s.", log_type="info")

        except ValueError as e:
            self._publish_feedback(f"ValueError: Could not set '{wait_seconds}' as new wait duration: {e}", log_type="error")
            return False
        
        except Exception as e:
            self._publish_feedback(f"ExceptionError: Could not set '{wait_seconds}' as new wait duration: {e}", log_type="error")
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
        
        if len(waypoint_name) != 3:
            self.get_logger().error("Waypoint must have exactly 3 values (x, y, yaw).")
            return False

        # set new joint position
        x, y, yaw = tuple(float(val) for val in new_waypoint)
        self.WAYPOINT_POSITIONS[waypoint_name] = (x, y, yaw)
        self.get_logger().info(f"Updated/Added new waypoint position: '{waypoint_name}'!")

        return True



def main(args=None):
    """
    Initializes the ROS 2 node and spins the RobotWaypointFollower.

    Args:
        args: Optional ROS 2 node arguments.
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