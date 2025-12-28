#!/usr/bin/env python3
"""
robot_arm_controller.py

Author:         Hugh Brennan
University:     University of Canberra

Project:        Pinapple Grand Challenge
Version:        2.0.0v
Created:        23/08/2025
Updated:        21/11/2025

Description:
    ROS 2 node for high-level robot arm control using MoveIt!.
    Provides an action server for arm movement and position commands.
"""
import time
import threading

from typing import Callable

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.action import ActionClient, ActionServer, GoalResponse
from rclpy.action.server import ServerGoalHandle
from rclpy.action.client import ClientGoalHandle
from rclpy.task import Future
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import MotionPlanRequest, Constraints, JointConstraint
from action_msgs.msg import GoalStatus
from std_msgs.msg import String

# --- Import for Custom Action ---
from pnp_interfaces.action import ArmController
from pnp_interfaces.msg import CommandArguments, ArmMoveOptions, WaitDuration



class RobotArmController(Node):
    """
    ROS 2 Node that acts as the robot arm control service.
    It communicates with MoveIt! via the MoveGroup Action Client 
    and exposes high-level methods for pick-and-place tasks.

    Args:
        Node (rclpy.node): node class from ROS 2 client library,
                           core node functionality is inherited from class.
    """
    # --- Class Constants ---
    NODE_NAME : str         = 'robot_arm_controller'
    PLANNING_GROUP : str    = 'tmr_arm'                 # match planning group (Robot Arm) name defined in MoveIt configuration
    ACTION_NAME : str       = '/move_action'            # match MoveIt server action name
    JOINT_NAMES : list[str] = ['joint_1','joint_2','joint_3','joint_4','joint_5','joint_6']

    JOINT_POSITIONS : dict[str, list[float]] = {        # target joint angles dictionary, in radians
        "default":          [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],                     # degrees:  00,  00,  00,  00,  00,  00
        "home":             [0.0, -0.7854, 1.7453, 0.7854, 0.0, 0.0],           # degrees:  00, -45, 100,  45,  00,  00
        "moving":           [0.0, -1.5708, 2.2689, 1.0123, 0.0, 0.0],           # degrees:  00, -90, 130,  58,  00,  00
        "carry_box":        [-0.6981, -1.5708, 2.2689, 1.0123, 0.0, 0.0],       # degrees: -40, -90, 130,  58,  00,  00
        "place_box":        [0.0, 0.5236, 1.2217, -0.1745, 1.5708, 0.0],        # degrees:  00,  30,  70, -10,  90,  00
        "small_pick_box":   [0.8378, 1.0123, 0.5934, 0.0, 1.5708, 0.0],         # degrees:  48,  58,  34,  00,  90,  00
        "medium_pick_box":  [0.4538, 0.3491, 1.3963, -0.1396, 1.5708, 0.0],     # degrees:  26,  20,  80,  -8,  90,  00
        "big_pick_box":     [0.0, 0.4887, 1.1170, 0.0, 1.5708, 0.0]             # degrees:  00,  28,  64,  00,  90,  00
    }
    STATE_LABELS : list[str]        = ["idle", "busy", "error"]
    STATUS_LABELS : list[str]       = ["READY", "ACCEPTED", "REJECTED", "EXECUTING", "SENDING", "SETTING", "WAITING", "COMPLETED", "FAILED"]
    SERVICE_CALL_TIMEOUT : float    = 5.0

    
    def __init__(self) -> None:
        """
        Sets up the ROS 2 node name,
        establishes the MoveGroup ActionClient to communicate with MoveIt!, 
        initializes the Action Server for the coordinator, and
        sets the internal state variables.
        """
        super().__init__(self.NODE_NAME)

        self._client : ActionClient = ActionClient(self, MoveGroup, self.ACTION_NAME)
        self._server : ActionServer = ActionServer(    # Create ROS 2 Action Server
            self,
            ArmController,
            f'/pnp/{self.NODE_NAME}',
            self._execute_callback,
            goal_callback=self._goal_callback
        )
        self._joints : list[float]  = self.JOINT_POSITIONS.get("default", [0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        self._arm_position : str    = "Unknown"
        self._wait_seconds : float  = 1.0
        self._state : int   = 0     # 0: "idle", 1: "busy", 3: "error"
        self._status : int  = 0     # 0: "READY", 1: "ACCEPTED", 2: "REJECTED", 3: "EXECUTING", 4: "SENDING", 5: "SETTING", 6: "WAITING", 7: "COMPLETED", 8: "FAILED"

        # store current goal handle
        self._goal_handle : ServerGoalHandle | None = None

        self.get_logger().info('Arm Controller node started. Waiting for MoveGroup action server...')

        if not self._client.wait_for_server(timeout_sec=self.SERVICE_CALL_TIMEOUT):
            self.get_logger().error('MoveGroup action server not available after 5s.')
            self._state = 3

            """ ADD CODE: attempt to clear error state """

        else:
            self.get_logger().info('MoveGroup action server ready.')

        # locks thread to ensure only one goal is being processed at a time
        self._goal_lock = threading.Lock()


    # --- Action Server Callbacks ---
    def _goal_callback(self, goal_request : ArmController.Goal) -> GoalResponse:
        """
        Accepts or rejects a new goal based on command validity and node status.

        Args:
            goal_request (ArmController.Goal): incoming goal request.

        Returns:
            GoalResponse: ACCEPT if the command is valid and node is ready, else REJECT.
        """
        self.get_logger().info(f'Accepting New Arm Action Goal: {goal_request.command}')
        # update internal goal status to ACCEPTED
        self._status = 1
        return GoalResponse.ACCEPT


    def _execute_callback(self, goal_handle : ServerGoalHandle) -> ArmController.Result:
        """
        Called when the goal is accepted.
        Runs accepted goal and returns the final result.

        Args:
            goal_handle (ServerGoalHandle): handle for the accepted goal.

        Returns:
            ArmController.Result: result of the action.
        """
        # update class goal handle
        self._goal_handle = goal_handle

        # retrieve request and prepare result
        request : ArmController.Goal = goal_handle.request
        result : ArmController.Result = ArmController.Result()

        # get callable command method
        method : Callable | None = getattr(self, request.command, None)

        # validate state and command
        if not self._check_state() or not self._check_callable(request.command):
            # update result and abort goal
            result.success = False
            result.state = self._state
            goal_handle.abort()
            self._goal_handle = None
            return result

        # update internal state to busy and feedback status to EXECUTING
        self._state = 1
        self._status = 3
        self._publish_feedback("Executing Arm goal...", "info")

        # execute goal
        result = self._run_goal(goal_handle)

        return result


    # --- Goal Handle ---
    def _run_goal(self, goal_handle : ServerGoalHandle) -> ArmController.Result:
        """
        Performs the synchronous/blocking work for the Action Goal.
        Parses the CommandArguments message to retrieve goal.

        Args:
            goal_handle (ServerGoalHandle): handle for the active action goal
            result (ArmController.Result): result message to populate

        Returns:
            ArmController.Result: result of the action (success flag and message).
        """
        request : ArmController.Goal = goal_handle.request
        result : ArmController.Result = ArmController.Result()

        # initialize success flag
        success : bool = False

        # initialize feedback variables
        status_msg : str = ""
        log_type : str   = "info"

        # lock goal execution to prevent concurrent access
        with self._goal_lock:
            try:
                # retrieve command (method) and arguments
                command : str           = request.command
                args : CommandArguments = request.command_arguments

                # get callable command method (existance validated by '_execute_callback()')
                method : Callable = getattr(self, command)

                # execute the method
                success = method(args)

            except Exception as e:
                status_msg = f"Arm goal failed. Exception Thrown: {e}"
                self.get_logger().error(status_msg)
                # update result flag
                success = False

            # final result
            result.success = success
            
            if success:
                status_msg = f"Success: Goal {command} completed."
                goal_handle.succeed()
                self._state = 0        # update internal state 'idle'
                self._status = 7       # update status to COMPLETED

            else:
                if not status_msg:
                    status_msg  = "Unknown Error: Arm goal failed."
                log_type    = "error"
                goal_handle.abort()
                self._state = 2        # update internal state 'error'
                self._status = 8       # update status to FAILED

            # update final result with node state  
            result.state = self._state

            # publish final feedback message
            self._publish_feedback(status_msg, log_type)
            self._goal_handle = None    # clear goal handle

        return result


    def _publish_feedback(self, msg : str = "", log_type : str = "") -> None:
        """
        Publishes feedback to the active goal handle.

        Args:
            msg (str, optional): main feedback message. Defaults to "".
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
        feedback : ArmController.Feedback   = ArmController.Feedback()
        feedback.status                     = self._get_status_name()
        feedback.status_msg                 = msg
        feedback.state                      = self._state

        # publish feedback message
        self._goal_handle.publish_feedback(feedback)


    def _get_state_name(self) -> str:
        """
        Takes the current state of the robot arm and returns its translated string representation.

        Returns:
            str: string of status name
        """
        return self.STATE_LABELS[self._state]


    def _get_status_name(self) -> str:
        """
        Takes the current status of the robot arm and returns its translated string representation.

        Returns:
            str: string of state name
        """
        return self.STATUS_LABELS[self._status]


    def _check_state(self) -> bool:
        """
        Checks if the node is in a state to accept new goals.

        Returns:
            bool: True if node is ready for new goals, False otherwise.
        """
        if self._state == 1:
            self._status = 2        # update status to REJECTED
            self._publish_feedback("Command Failed: Robot Arm Controller is busy. Rejecting new goal.", "warn")
            return False
        
        if self._state == 3:
            self._status = 2        # update status to REJECTED
            self._publish_feedback("Command Failed: Error with Robot Arm. Ignoring new command(s).", "error")
            return False
        
        return True


    def _check_callable(self, command : str) -> bool:
        """
        Checks if the requested command is a valid callable method of this class.

        Args:
            command (str): name of the command/method to check.

        Returns:
            bool: True if the command is valid and callable, False otherwise.
        """
        method : Callable | None = getattr(self, command, None)
        if not method or not callable(method):
            self._status = 2        # update status to REJECTED
            self._publish_feedback("Command Failed: Unknown command received. Rejecting new goal.", "warn")
            return False
        
        return True


    # _clear_error is currently depricated
    def _clear_error(self) -> None:
        """
        Attempts to clear an error state by re-checking Nav2 server readiness.
        """
        self.get_logger().warn('Attempting to clear error status by re-checking MoveGroup action server readiness...')

        if self._client.wait_for_server(timeout_sec=self.SERVICE_CALL_TIMEOUT):
            self.get_logger().info('MoveGroup action server is READY. Resetting state to IDLE (0).')
            self._state = 0


    # --- Method to Send a Goal and Wait for Result ---
    def _send_and_wait(self) -> bool:
        """
        Sends the current goal to MoveIt! and waits for the result.

        Returns:
            bool: True if the movement succeeded, False otherwise.
        """
        # 1. Prepare Goal
        goal : MoveGroup.Goal   = self._goal_from_joints()
        send_future : Future    = self._client.send_goal_async(goal)

        # 2. Send and Wait for Goal Response
        self._status = 4        # update status to SENDING
        self._publish_feedback("Sending MoveGroup Goal to server...", "info")

        while not send_future.done():
            time.sleep(0.05)

        # 3. Check for Goal Acceptance
        handle : ClientGoalHandle | None = send_future.result()
        if not handle or not handle.accepted:
            self._state = 3
            self._status = 8    # update status to FAILED
            # publish new feedback message
            self._publish_feedback("MoveGroup Goal rejected by server.", "error")
            return False
        
        # 4. Wait for MoveIt Goal Complete
        self._status = 6        # update status to WAITING
        self._publish_feedback("MoveGroup Goal accepted. Waiting for results...", "info")

        result_future : Future = handle.get_result_async()
        while not result_future.done():
            time.sleep(0.05)

        result : MoveGroup.Result | None = result_future.result()

        # 5. Check for Successful Goal Status (GoalStatus.SUCCEEDED or 4); Log and Return Result
        success : bool = bool(result and result.status == GoalStatus.STATUS_SUCCEEDED)
        if success:
            pass
        else:
            self._publish_feedback(f"Arm movement failed with status: {result.status}", "error")
            self._arm_position = "Unknown"
            self._state = 3

        return success


    # --- Helper Methods to Build a MoveGroup Goal ---
    def _get_joints(self, key : str) -> bool:
        """
        Retrieves a joint position from the dictionary and updates the internal '_joints' attribute.

        Args:
            key (str): key (name) of the joint position to load.

        Returns:
            bool: True if the key exists and was loaded, False otherwise.
        """
        if key not in self.JOINT_POSITIONS:
            self._state = 3
            # publish new feedback message
            self._publish_feedback(f"Unknown Joint Set: {key}", "error")
            return False
        
        self._joints = self.JOINT_POSITIONS[key]

        # publish new feedback message
        self._publish_feedback(f"Selected Joints ({key}): {self._joints}", "info")

        return True


    def _goal_from_joints(self) -> MoveGroup.Goal:
        """
        Creates a MoveGroup action goal message from the currently set internal '_joints' list.

        Returns:
            MoveGroup.Goal: fully constructed goal message, ready to be sent to the MoveGroup Action Server.
        """
        goal : MoveGroup.Goal   = MoveGroup.Goal()
        req : MotionPlanRequest = MotionPlanRequest()

        req.group_name                      = self.PLANNING_GROUP
        req.num_planning_attempts           = 10
        req.allowed_planning_time           = 5.0
        req.max_velocity_scaling_factor     = 0.5
        req.max_acceleration_scaling_factor = 0.5

        cs : Constraints = Constraints()
        for name, val in zip(self.JOINT_NAMES, self._joints):
            jc : JointConstraint = JointConstraint()
            jc.joint_name       = name
            jc.position         = val
            jc.tolerance_above  = 0.01
            jc.tolerance_below  = 0.01
            jc.weight           = 1.0
            cs.joint_constraints.append(jc)

        req.goal_constraints.append(cs)
        goal.request = req
        goal.planning_options.plan_only = False  # plan + execute

        return goal


    # --- Arm Commands ---
    def _arm_move(self, args : CommandArguments) -> bool:
        """
        Moves the arm to chosen predefined position.

        Args:
            args (CommandArguments): command arguments containing chosen pre-defined joint position from JOINT_POSITIONS (e.g., 'default', 'home').

        Returns:
            bool: True if move was successful, False otherwise.
        """
        # retrieve target position
        pos : str = args.arm_goal.target_position
        pos = pos.lower()
        if not self._get_joints(pos):
            # _get_joints handles error logging
            return False
        
        # update current arm position
        self._arm_position : str = pos

        # publish new feedback message
        self._publish_feedback(f"Moving arm to {pos.upper()} position...", "info")

        # send and retrieve goal result
        result : bool = self._send_and_wait()

        # pause in current position
        self._wait_at_position()

        return result


    def _wait_at_position(self, args : CommandArguments | None = None) -> bool:
        """
        Pauses execution for a specified duration at the current arm position.

        Args:
            args (CommandArguments | None): arguments containing WaitDuration. If None, uses class default. Defaults to None.

        Returns:
            bool: True after sleep complete.
        """
        # use class default if none passed
        if args is None or not isinstance(args, CommandArguments):
            self._publish_feedback(f"No duration passed to '_wait_at_position', using class preset: {self._wait_seconds}", "info")
            wait_seconds : float = self._wait_seconds
        else:
            wait_seconds : float = args.wait_goal.duration
            try:
                wait_seconds = float(wait_seconds)
            except ValueError as e:
                self._publish_feedback(f"ValueError: Could not use '{wait_seconds}' as duration for '_wait_at_position': {e}", "error")
                return False

        # use class default
        if wait_seconds == -1.0:
            wait_seconds = self._wait_seconds

        # skip wait
        if wait_seconds <= 0.0:
            return True
        
        # publish new feedback message
        self._publish_feedback(f"Waiting {wait_seconds:.1f} seconds in position {self._arm_position}...", "info")

        # wait for defined duration
        time.sleep(wait_seconds)

        return True


    # --- Getters ---
    def _get_joint_position_names(self) -> list[str]:
        """
        Retrieves the list of available joint position names (keys) stored in JOINT_POSITIONS.

        Returns:
            list[str]: list of all predefined position names.
        """
        return list(self.JOINT_POSITIONS.keys())
    

    def _get_joint_values(self, position_name : str) -> list[float] | None:
        """
        Retrieves the joint angle values (in radians) from a position name.

        Args:
            position_name (str): string key for the joint position to retrieve.

        Returns:
            list[float] | None: list of 6 float values if the key is found, or None.
        """
        # _get_joints attempts to load the position into self._joints
        if not self._get_joints(position_name):
            return
        
        return self._joints


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
                self._publish_feedback(f"Warning: Value must be > or = 0.0. Ignoring attempt to set wait duration '{wait_seconds}'", "warn")
                return False
            
            # update duration
            self._wait_seconds = wait_seconds

            self._publish_feedback(f"Default wait duration set to {wait_seconds}s.", "info")

        except ValueError as e:
            self._publish_feedback(f"ValueError: Could not set '{wait_seconds}' as new wait duration: {e}", "error")
            return False
        
        except Exception as e:
            self._publish_feedback(f"ExceptionError: Could not set '{wait_seconds}' as new wait duration: {e}", "error")
            return False
        
        return True


    def _set_joint_position(self, new_joints : list[float], position_name : str) -> bool:
        """
        Adds or updates a named joint position in the JOINT_POSITIONS dictionary.

        Args:
            new_joints (list[float]):   list of 6 float values representing the target joint angles (in radians).
            position_name (str):        unique or pre-existing name to assign or update this joint position.

        Returns:
            bool: True if position was successfully added/updated, False otherwise.
        """
        # new position validation checks
        if len(new_joints) != 6:
            self.get_logger().error(f"Joint list size invalid. Expected {len(self.JOINT_NAMES)} but got {len(new_joints)}.")
            return False
        
        if not all(isinstance(joint, (int, float)) for joint in new_joints):
            self.get_logger().error("Joint values must be floats or integers.")
            return False
        
        if not isinstance(position_name, str) or not position_name.strip():
            self.get_logger().error("Position name must be a non-empty string.")
            return False
        
        # set new joint position
        self.JOINT_POSITIONS[position_name] = [float(j) for j in new_joints]
        self.get_logger().info(f"Updated/Added new joint position: '{position_name}'!")

        return True



def main(args=None) -> None:
    """
    Initializes the ROS 2 node and spins the RobotArmController.

    Args:
        args: Optional ROS 2 node arguments.
    """
    rclpy.init(args=args)

    node = RobotArmController()

    executor = MultiThreadedExecutor()
    executor.add_node(node)
    
    try:
        # Spin the node to keep the ActionClient alive and ready for calls
        node.get_logger().info('RobotArmController is now spinning and ready to accept commands!')
        executor.spin()
    except KeyboardInterrupt:
        node.get_logger().info('Arm Commander stopped by user (KeyboardInterrupt).')
    finally:
        node.get_logger().info('Arm sequence complete. Shutting down.')
        executor.remove_node(node)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()