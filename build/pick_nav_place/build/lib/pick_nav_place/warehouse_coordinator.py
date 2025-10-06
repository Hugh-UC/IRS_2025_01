#!/usr/bin/env python3
import math
import time
import json 

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from std_msgs.msg import String, Empty 





# self.get_logger().info('Sequence complete ✅')