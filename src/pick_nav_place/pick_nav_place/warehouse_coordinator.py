#!/usr/bin/env python3
import math
import time
import json 

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import String, Empty

# Import the service classes
from pick_nav_place.robot_arm_controller import RobotArmController
from pick_nav_place.robot_waypoint_follower import RobotWaypointFollower
from pick_nav_place.plc_hmi_listener import PLCHmiListener


class WarehouseCoordinator(Node):
    def __init__(self):
        """
        Initializes the Warehouse Coordinator node and its service clients.
        Sets up the main timer for the coordination logic.
        """
        super().__init__('warehouse_coordinator')
        self.get_logger().info('Initializing Warehouse Coordinator...')

        # instantiate the service nodes
        self.arm_controller : RobotArmController = RobotArmController()
        self.nav_controller : RobotWaypointFollower = RobotWaypointFollower()
        self.hmi_listener : PLCHmiListener = PLCHmiListener()

        # coordination state variables
        self.is_busy: bool = False
        
        self.nav_controller.set_wait_seconds(5.0)
        self.arm_controller.set_wait_seconds(8.0)
        

        self.hmi_subscription = self.create_subscription(
            String,
            '/pnp/hmi_status',
            self._hmi_data_callback, # <-- The new main trigger
            10
        )
        self.get_logger().info('Coordinator started. Now listening for PLC/HMI data...')

    

    def _hmi_data_callback(self, msg: String) -> None:
        data = json.loads(msg.data)

        if self.is_busy:
            self.get_logger().info("Robot is busy. Ignoring new HMI message.")
            return
        
        signal : bool = data.get("signal_new_box", False)

        self.get_logger().info(f"New Box Signaled: {signal}")

        if signal is False:
            return
        
        self.is_busy = True
        self.get_logger().info(f"Robot is now busy!")
        execution_success : bool = self._execute_coordinator()
        self.is_busy = False

        if execution_success:
            self.get_logger().info("✅ Full pick, nav, and place sequence successfully completed!")
        else:
            self.get_logger().error("❌ Coordination cycle failed. The system will wait for the next unique PLC/HMI data update to retry.")


    # --- Main Logic Loop ---
    def _execute_coordinator(self) -> bool:
        self.get_logger().info("--- Starting Coordination Sequence...")
        self.arm_controller.set_wait_seconds(4.0)
        self.nav_controller.set_wait_seconds(2.0)

        # -- move to conveyor --
        if not self.arm_controller.arm_move("moving"):
            self.get_logger().error("Failed to move arm to 'moving' position.")
            return False

        self.nav_controller.wait_at_waypoint(0.5)

        if not self.nav_controller.go_to_waypoint("conveyor"):
            self.get_logger().error("Failed, robot did not success reach 'conveyor' waypoint.")
            return False

        self.nav_controller.wait_at_waypoint()
        
        # wait for box to be ready (Encapsulated Polling Logic)
        box_ready, box_size_key, box_location = self._wait_for_box_ready()
        
        self.get_logger().info(f"Box is ready to be picked from conveyor position '{box_location}'.")

        # -- pick up box --
        if not self.arm_controller.arm_pick_box(box_size_key):
            self.get_logger().error(f"Failed to move arm for picking {box_size_key}.")
            return False
        
        # -- carry box --
        if not self.arm_controller.arm_move("carry_box"):
            self.get_logger().error("Failed to move arm to 'carry_box' position.")
            return False
        
        self.get_logger().info(f"Box has been sucessfully picked from conveyor position '{box_location}'.")

        # -- move to drop-off
        if not self.nav_controller.go_to_waypoint("drop-off"):
            self.get_logger().error("Failed, robot did not reach 'drop-off' waypoint.")
            return False
        
        self.nav_controller.wait_at_waypoint(1.0)

        if not self.arm_controller.arm_move("place_box"):
            self.get_logger().error("Failed to move arm to 'place_box' position.")
            return False

        self.arm_controller.wait_at_position()

        if not self.arm_controller.arm_move("moving"):
            self.get_logger().error("Failed to move arm to 'moving' position.")
            return False
        
        self.nav_controller.wait_at_waypoint(1.0)

        if not self.nav_controller.go_to_waypoint("home"):
            self.get_logger().error("Failed, robot did not success reach 'home' waypoint.")
            return False
        
        return True


    def _wait_for_box_ready(self) -> tuple[bool, str | None, str | None]:
        """
        Polls the HMI listener to wait for a box location to finalise and match the expected location.
        Returns: (success: bool, box_size_key: str, box_location: str)
        """
        MAX_WAIT_TIME = 5.0

        # retrieve box weight and determine expected location
        box_weight: str | None = self.hmi_listener.get_box_weight()

        if not box_weight:
            self.get_logger().error("Box weight was not available. Cannot determine expected location.")
            return False, None, None

        box_size_key, expected_location = self._retrieve_box_size_key(box_weight)

        if not box_size_key or not expected_location:
            self.get_logger().error("Failed to map box weight to a known size key and expected location.")
            return False, None, None
        
        # polling loop with 5 second timeout
        start_time = time.time()
        final_box_location: str = ""

        self.get_logger().info(f"Waiting for box location to be finalised (Expected: '{expected_location}') with a {MAX_WAIT_TIME} second timeout...")

        while time.time() - start_time < MAX_WAIT_TIME:
            current_box_location: str | None = self.hmi_listener.get_box_location()

            if current_box_location:
                
                if expected_location == current_box_location:
                    # SUCCESS: Return True, size, and confirmed location
                    self.get_logger().info(f"Box location '{current_box_location}' confirmed and matches expected location.")
                    
                    return True, box_size_key, current_box_location
                else:
                    # FAILURE: Location mismatch
                    self.get_logger().error(f"Box location finalised to '{current_box_location}', but expected '{expected_location}'. Aborting pick.")

                    return False, None, None

            time.sleep(0.1)
        
        # 3. Timeout failure
        self.get_logger().error(f"Timed out after {MAX_WAIT_TIME} seconds waiting for final box location. Aborting pick sequence.")
        return False, None, None


    def _retrieve_box_size_key(self, weight_str: str) -> tuple[str | None, str | None]:
        """
        Parses the weight string (e.g., '6 kg') to determine the box size key ('small', 'medium', 'big').
        """
        try:
            # 1. Extract the number from the string
            weight_val : float = float(weight_str.lower().replace(' kg', '').strip())
        except ValueError:
            self.get_logger().error(f"Failed to parse weight string: {weight_str}.")
            return None, None
        
        # 2. Map weight to size key (Assumed thresholds: <10=Small, <20=Medium, >=20=Big)
        if weight_val <= 5.0:
            return ("small", "C")
        elif weight_val <= 10.0:
            return ("medium", "B")
        elif weight_val > 10.0:
            return ("big", "A")
        
        self.get_logger().error(f"Weight {weight_val} kg does not match a known box size range.")
        return None, None




    def test_arm(self):
        self.arm_controller.wait_at_position()

        if not self.arm_controller.arm_move("moving"):
            self.get_logger().error("Failed to move arm to 'moving' position.")
            return False
        
        self.arm_controller.wait_at_position()

        if not self.arm_controller.arm_move("default"):
            self.get_logger().error("Failed to move arm to 'default' position.")
            return False
        
    
    def test_waypoint(self):
        self.nav_controller.wait_at_waypoint()

        if not self.nav_controller.go_to_waypoint("conveyor"):
            self.get_logger().error("Failed to move robot to 'conveyor' waypoint.")
            return False
        
        self.nav_controller.wait_at_waypoint()

        if not self.nav_controller.go_to_waypoint("home"):
            self.get_logger().error("Failed to move robot to 'home' waypoint.")
            return False
        
        self.get_logger().info('Sequence complete ✅')


def main(args=None):

    rclpy.init(args=args)
    node = WarehouseCoordinator()

    executor = MultiThreadedExecutor()
    executor.add_node(node)
    executor.add_node(node.arm_controller)
    executor.add_node(node.nav_controller)
    executor.add_node(node.hmi_listener)

    #node.test_arm()
    #node.test_waypoint()

    try:
        node.get_logger().info('Starting coordination executor. All nodes spun up.')
        executor.spin()
    except KeyboardInterrupt:
        node.info('Coordinator stopped by user (KeyboardInterrupt).')
    finally:
        node.get_logger().info('Coordinator shutting down.')
        # Cleanly destroy all nodes
        node.arm_controller.destroy_node()
        node.nav_controller.destroy_node()
        node.hmi_listener.destroy_node()
        node.destroy_node()
        rclpy.shutdown()