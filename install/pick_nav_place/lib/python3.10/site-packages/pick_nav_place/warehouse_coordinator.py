#!/usr/bin/env python3
import math
import time
import json
from typing import Any

import rclpy
import rclpy.time
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import String, Empty
from std_srvs.srv import Empty as EmptySrv
from geometry_msgs.msg import PoseWithCovarianceStamped, Pose, Point, Quaternion


class WarehouseCoordinator(Node):
    COORD_STATES : dict[int, str] = {
        0: "IDLE",
        1: "ARM_TO_MOVING",
        2: "NAV_TO_CONVEYOR",
        3: "ARM_PICK_BOX",
        4: "ARM_TO_CARRY",
        5: "NAV_TO_DROPOFF",
        6: "ARM_PLACE_BOX",
        7: "ARM_TO_MOVING",
        8: "NAV_TO_HOME",
        9: "ARM_TO_HOME",
        10: "COMPLETE",
        99: "ERROR",
    }
    ARM : str = "robot_arm_controller"
    NAV : str = "robot_waypoint_follower"


    def __init__(self):
        """
        Initializes the Warehouse Coordinator node and its service clients.
        Sets up the main timer for the coordination logic.
        """
        super().__init__('warehouse_coordinator')
        self.get_logger().info('Initializing Warehouse Coordinator...')

        # coordination state variables
        self._current_box_size : str        = "Null"
        self._current_box_location : str    = "Null"
        self._hmi_status : str              = "idle"
        self._nav_status : str              = "idle"
        self._arm_status : str              = "idle"
        self._coord_state : int             = 0

        self.initialised : bool             = False

        # ROS2 Topic Publisher
        self._coordinator_pub = self.create_publisher(          # dedicated coordinator topic
            String, 
            '/pnp/coordinator',
            10
        )

        # ROS2 Topic Subscriptions
        self.hmi_status_sub = self.create_subscription(         # plc_hmi_listener.py
            String,
            '/pnp/hmi_status',
            self._handle_status_update,
            10
        )
        self.get_logger().info('Now listening for PLC/HMI data...')

        self._waypoint_status_sub = self.create_subscription(   # robot_waypoint_follower.py
            String,
            '/pnp/waypoint_follow_status',
            self._handle_status_update,
            10
        )
        self.get_logger().info('Now listening for Robot Navigation status...')

        self._arm_status_sub = self.create_subscription(        # robot_arm_controller.py
            String,
            '/pnp/arm_status',
            self._handle_status_update,
            10
        )
        self.get_logger().info('Now listening for Robot Arm status...')


        self._initial_pose_pub = self.create_publisher(
            PoseWithCovarianceStamped, 
            '/initialpose',
            10
        )

        # pause for all nodes to initialise
        time.sleep(12.0)

        # run initialisation sequence
        self._initialisation_sequence()

        # log startup complete
        self.get_logger().info('Coordinator startup complete!')

    
    def _publish_command(self, target : str, method : str, value : Any = None) -> None:
        if value is None:
            value = ""

        payload : dict[str, Any] = {
            "node": target,
            "command": {
                "method": method,
                "value": value
            },
            "timestamp": int(self.get_clock().now().nanoseconds / 1e9),
        }

        json_payload : str = json.dumps(payload)

        msg : String = String()
        msg.data = json_payload
        
        self.get_logger().info(f"COMMAND -> {target}.{method}({value})")
        self._coordinator_pub.publish(msg)


    def _handle_status_update(self, msg : String) -> None:
        try:
            data = json.loads(msg.data)

            data_node = data.get("node")
            data_status = data.get("status")

            # 1. Update internal status cache
            match data_node:
                case "plc_hmi_listener":
                    self._hmi_status = data_status
                    if 'current_target' in data:
                        self._current_box_size = data['current_target'].get('box_size', "Null")
                        self._current_box_location = data['current_target'].get('location', "Null")

                case "robot_waypoint_follower":
                    self._nav_status = data_status
                case "robot_arm_controller":
                    self._arm_status = data_status
            
            # 2. Handle Error State
            if not self._error_check():
                return

            self._execute_coordinator()

        except Exception as e:
            self.get_logger().error(f"❌ Failed to parse status update: {e}\nRaw msg={msg.data}")
            self._coord_state = 99
            self._check_state()


    def _error_check(self) -> bool:
        if self._hmi_status == "error" or self._arm_status == "error" or self._nav_status == "error":
            self.get_logger().error(f"FATAL ERROR detected. Current State: {self.COORD_STATES.get(self._coord_state)}.")
            rclpy.shutdown()

            return False
        
        if self._coord_state == 99:
            # Clear error state
            self.get_logger().info(f"Error has cleared! Resetting to IDLE.")
            self._coord_state = 0

        return True


    def _initialisation_sequence(self) -> None:
        self.get_logger().info("Running Initialisation Sequence...")

        self._reinitialize_amcl()

        self._publish_command("robot_waypoint_follower", "_spin", 1.5708)
        self._publish_command("robot_waypoint_follower", "_spin", 1.5708)
        self._publish_command("robot_waypoint_follower", "_spin", 1.5708)
        self._publish_command("robot_waypoint_follower", "_spin", 1.5708)

        self.get_logger().info("Initialisation Complete.")


    def _reinitialize_amcl(self) -> bool:
        """
        Calls the AMCL service to reinitialize global localization.
        This forces AMCL to spread particles across the entire map.
        """
        self.get_logger().warn("⚠️ Calling AMCL service to reinitialize global localisation...")

        GLOBAL_SEARCH_VARIANCE = 99999.0

        pose_msg = PoseWithCovarianceStamped()
        pose_msg.header.stamp = rclpy.time.Time().to_msg()
        pose_msg.header.frame_id = 'map'

        # 1. Set Position
        pose_msg.pose.pose = Pose(
            position=Point(x=0.0, y=0.0, z=0.0), 
            orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
        )

        pose_msg.pose.covariance = [
            GLOBAL_SEARCH_VARIANCE, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, GLOBAL_SEARCH_VARIANCE, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, GLOBAL_SEARCH_VARIANCE # Yaw (index 35)
        ]

        self._initial_pose_pub.publish(pose_msg)


    # --- Main Logic Loop ---
    def _execute_coordinator(self) -> None:
        # -- Start Sequence ---
        self.get_logger().info(f"CURRENT EXECUTION STEP: {self.COORD_STATES.get(self._coord_state)}")

        # State 0: 
        if self._coord_state == 0 and (self._hmi_status == "busy" or self._hmi_status == "ready"):
            self.get_logger().info("Starting Coordination Sequence...")

            self._publish_command(self.ARM, "_set_wait_seconds", 5.0)
            self._publish_command(self.NAV, "_set_wait_seconds", 5.0)

            self.get_logger().info(f"STATUS READY: Box {self._current_box_size} at {self._current_box_location} detected. Starting sequence.")
            self._coord_state = 1
            self._execute_coordinator()
            return

        # State 1:
        if self.validate_status(1, arm="idle"):
            self._publish_command("robot_arm_controller", "_arm_move", "moving")
            self._coord_state = 2
            return

        # State 2:
        if self.validate_status(2, nav="idle", arm="idle"):
            self._publish_command("robot_waypoint_follower", "_go_to_waypoint", "conveyor")
            self._coord_state = 3
            return

        # State 3:
        if self.validate_status(3, nav="idle", arm="idle", hmi="ready"):
            self._publish_command("robot_arm_controller", "_arm_pick_box", self._current_box_size)
            self._coord_state = 4
            return
        # handle box deleted or conveyor error
        elif self.validate_status(3, hmi="idle"):
            self._publish_command("robot_waypoint_follower", "_go_to_waypoint", "home")
            self._coord_state == 0
            return

        # State 4:
        if self.validate_status(4, arm="idle"):
            self._publish_command("robot_arm_controller", "_arm_move", "carry_box")
            self._coord_state = 5
            return

        # State 5:
        if self.validate_status(5, nav="idle", arm="idle"):
            self._publish_command("robot_waypoint_follower", "_go_to_waypoint", "drop-off")
            self._coord_state = 6
            return

        # State 6:
        if self.validate_status(6, nav="idle", arm="idle"):
            self._publish_command("robot_arm_controller", "_arm_move", "place_box")
            self._coord_state = 7
            return

        # State 7:
        if self.validate_status(7, nav="idle", arm="idle"):
            self._publish_command("robot_arm_controller", "_arm_move", "moving")
            self._coord_state = 8
            return

        # State 8:
        if self.validate_status(8, nav="idle", arm="idle"):
            self._publish_command("robot_waypoint_follower", "_go_to_waypoint", "home")
            self._coord_state = 9
            return

        # State 9:
        if self.validate_status(9, nav="idle", arm="idle"):
            self._publish_command("robot_arm_controller", "_arm_move", "home")
            self._coord_state = 10
            return

        # State 10:
        if self.validate_status(10, nav="idle", arm="idle"):
            self.get_logger().info("✅ Full pick, nav, and place sequence successfully completed and arm is home! Back to IDLE.")
            self._coord_state = 0
        
        return


    def validate_status(self, coord_state : int, nav : str = "", arm : str = "", hmi : str = "") -> bool:
        validate : bool = True

        if coord_state != self._coord_state:
            validate = False
        
        if nav and self._nav_status != nav:
            validate = False

        if arm and self._arm_status != arm:
            validate = False

        if hmi and self._hmi_status != hmi:
            validate = False

        return validate



def main(args=None):

    rclpy.init(args=args)
    node = WarehouseCoordinator()

    executor = MultiThreadedExecutor()
    executor.add_node(node)

    #node.test_arm()
    #node.test_waypoint()

    try:
        node.get_logger().info('Starting coordination executor. All nodes spun up.')
        executor.spin()
    except KeyboardInterrupt:
        node.info('Coordinator stopped by user (KeyboardInterrupt).')
    finally:
        node.get_logger().info('Coordinator shutting down.')
        node.destroy_node()
        rclpy.shutdown()