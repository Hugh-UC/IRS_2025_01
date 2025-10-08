#!/usr/bin/env python3
import math
import time
import json 

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
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

        # 1. Instantiate the service nodes (must be run within this node's context)
        # We don't spin them individually; we call their methods within our timer loop.
        self.arm_controller : RobotArmController = RobotArmController()
        self.nav_controller : RobotWaypointFollower = RobotWaypointFollower()
        self.hmi_listener : PLCHmiListener = PLCHmiListener()

        
        self.nav_controller.set_wait_seconds(5.0)
        self.arm_controller.set_wait_seconds(8.0)

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

        if not self.nav_controller.go_to_conveyor():
            self.get_logger().error("Failed to move robot to 'conveyor' waypoint.")
            return False
        
        self.nav_controller.wait_at_waypoint()

        if not self.nav_controller.go_home():
            self.get_logger().error("Failed to move robot to 'home' waypoint.")
            return False
        
        self.get_logger().info('Sequence complete ✅')


def main(args=None):

    rclpy.init(args=args)
    node = WarehouseCoordinator()

    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(node)
    executor.add_node(node.arm_controller)
    executor.add_node(node.nav_controller)
    executor.add_node(node.hmi_listener)

    node.test_arm()

    node.test_waypoint()

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