#!/usr/bin/env python3
import time

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.action.client import ClientGoalHandle
from rclpy.task import Future
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import MotionPlanRequest, Constraints, JointConstraint
from action_msgs.msg import GoalStatus
from std_msgs.msg import String


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
    JOINT_NAMES : list[str] = ['joint_1','joint_2','joint_3','joint_4','joint_5','joint_6']
    PLANNING_GROUP : str    = 'tmr_arm'                 # match planning group (Robot Arm) name defined in MoveIt configuration
    ACTION_NAME : str       = '/move_action'            # match MoveIt server action name

    JOINT_POSITIONS : dict[str, list[float]] = {        # target joint angles dictionary, in radians
        "default": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        "home": [0.0, -0.7854, 1.7453, 0.7854, 0.0, 0.0],
        "moving": [0.0, -1.5708, 2.2689, 1.0123, 0.0, 0.0],
        "carry_box": [-0.6981, -1.5708, 2.2689, 1.0123, 0.0, 0.0],
        "place_box": [0.0, 0.5236, 1.2217, 1.2217, 0.0, 0.0],
        "sm_pick_box": [0.8378, 1.0123, 0.5934, 0.0, 1.5708, 0.0],      # degrees: 48, 58, 34, 00, 90, 00
        "mm_pick_box": [0.4538, 0.3491, 1.3963, -0.1396, 1.5708, 0.0],  # degrees: 26, 20, 80, -8, 90, 00
        "bg_pick_box": [0.0, 0.4887, 1.1170, 0.0, 1.5708, 0.0]          # degrees: 00, 28, 64, 00, 90, 00
    }
    BOX_SIZES : dict[str, str] = {"small": "sm", "medium": "mm", "big": "bg"}

    
    def __init__(self):
        """
        Sets up the ROS 2 node name,
        establishes the MoveGroup ActionClient to communicate with MoveIt!,
        and initializes the internal state variables.
        """
        super().__init__('robot_arm_controller')
        self._client : ActionClient = ActionClient(self, MoveGroup, self.ACTION_NAME)
        self._joints : list[float]  = self.JOINT_POSITIONS.get("default", [0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        self._wait_seconds : float  = 1.0

        self.arm_status_pub = self.create_publisher(
            String, 
            '/pnp/arm_status',    # dedicated topic name
            10
        )

        self.get_logger().info('Waypoint Follower node started. Waiting for MoveGroup action server...')


    # --- Method to send a goal and wait for result ---
    def _send_and_wait(self) -> bool:
        """
        [PROTECTED HELPER] Sends the current goal to MoveIt! and waits for the result.

        Returns:
            bool: True if the movement succeeded, False otherwise.
        """
        if not self._client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('MoveGroup action server not available after 5s.')
            return False
        else:
            self.get_logger().info('MoveGroup action server ready.')

        goal : MoveGroup.Goal   = self._goal_from_joints()
        send_future : Future    = self._client.send_goal_async(goal)

        rclpy.spin_until_future_complete(self, send_future)

        handle : ClientGoalHandle | None = send_future.result()

        if not handle or not handle.accepted:
            self.get_logger().error('MoveGroup Goal rejected by server.')
            return False
        
        res_future : Future = handle.get_result_async()

        rclpy.spin_until_future_complete(self, res_future)

        res = res_future.result()

        # check for successful goal status (GoalStatus.SUCCEEDED or 4)
        success : bool = bool(res and res.status == GoalStatus.STATUS_SUCCEEDED)
        if success:
             self.get_logger().info('Arm movement successful.')
        else:
             self.get_logger().error(f'Arm movement failed with status: {res.status}')

        return success


    def _get_joints(self, key : str) -> bool:
        """
        [PROTECTED HELPER] Retrieves a joint position from the dictionary and updates the internal '_joints' attribute.

        Args:
            key (str): key (name) of the joint position to load.

        Returns:
            bool: True if the key exists and was loaded, False otherwise.
        """
        if key not in self.JOINT_POSITIONS:
            self.get_logger().error(f"Unknown Joint Set: {key}")
            return False
        
        self._joints = self.JOINT_POSITIONS[key]

        self.get_logger().info(f"Selected Joints: {self._joints}")              # REMOVE!!!!!

        return True

    
    def wait_at_position(self) -> None:
        """
        Pauses execution for a specified duration at the current arm position.

        Returns:
            None: sleep only method.
        """
        self.get_logger().info(f'Waiting {self._wait_seconds:.1f} seconds...')
        time.sleep(self._wait_seconds)


    # --- Helper method to build a MoveGroup Goal ---
    def _goal_from_joints(self) -> MoveGroup.Goal:
        """
        [PROTECTED HELPER] Creates a MoveGroup action goal message from the currently set internal '_joints' list.

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
    def arm_move(self, pos : str) -> bool:
        """
        Moves the arm to chosen predefined position.

        Args:
            pos (str): string of chosen pre-defined joint position in JOINT_POSITIONS (e.g., 'default', 'home').

        Returns:
            bool: True if move was successful, False otherwise.
        """
        pos = pos.lower()
        if not self._get_joints(pos):
            # _get_joints handles error logging
            return False
        
        self.get_logger().info(f"Moving arm to {pos.upper()} position...")

        return self._send_and_wait()


    def arm_pick_box(self, box_size : str) -> bool:
        """
        Moves the arm to position for picking a box of the specified size.

        Args:
            box_size (str): size key ('small', 'medium', or 'big').

        Returns:
            bool: True if move was successful, False otherwise.
        """
        box_size = box_size.strip().lower()

        pos_name : str | None = self.BOX_SIZES.get(box_size)

        if pos_name is None:
            self.get_logger().error(f"Invalid box size key: {box_size}")
            return False
        
        pos : str = f"{pos_name}_pick_box"

        self.get_logger().info(f"Moving arm to {box_size.upper()} box PICK position...")

        if not self._get_joints(pos):
            # _get_joints handles error logging
            return False

        return self._send_and_wait()


    # --- Getters ---
    def get_joint_position_names(self) -> list[str]:
        """
        Retrieves the list of available joint position names (keys) stored in JOINT_POSITIONS.

        Returns:
            list[str]: list of all predefined position names.
        """
        return list(self.JOINT_POSITIONS.keys())
    

    def get_joint_values(self, position_name : str) -> list[float]:
        """
        Retrieves the joint angle values (in radians) from a position name.

        Args:
            position_name (str): string key for the joint position to retrieve.

        Returns:
            list[float]: list of 6 float values if the key is found, False otherwise.
        """
        # _get_joints attempts to load the position into self._joints
        if not self._get_joints(position_name):
            return False
        
        return self._joints


    # --- Setters ---
    def set_wait_seconds(self, seconds : float):
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


    def set_joint_position(self, new_joints : list[float], position_name : str) -> bool:
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
    


def main(args=None):
    """
    Initializes the ROS 2 node and spins the RobotArmController.
    """
    rclpy.init(args=args)

    node = RobotArmController()
    
    try:
        # Spin the node to keep the ActionClient alive and ready for calls
        node.get_logger().info('RobotArmController is now spinning and ready to accept commands!')
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Arm Commander stopped by user (KeyboardInterrupt).')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()