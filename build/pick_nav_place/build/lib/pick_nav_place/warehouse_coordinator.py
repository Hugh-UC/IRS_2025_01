#!/usr/bin/env python3
import math
import time
import json
import threading
from typing import Any

import rclpy
import rclpy.time
from rclpy.time import Time
from rclpy.duration import Duration
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import String, Empty
from std_srvs.srv import Empty as EmptySrv
from geometry_msgs.msg import PoseWithCovarianceStamped, Pose, Point, Quaternion
from nav2_msgs.srv import ClearEntireCostmap


class WarehouseCoordinator(Node):
    # Sequence States
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
        101: "CLEAR_COSTMAPS"
    }
    # Define Node Names
    NAV : str = "robot_waypoint_follower"
    ARM : str = "robot_arm_controller"
    PLC : str = "plc_hmi_listener"
    COR : str = "warehouse_coordinator"
    NODE_STATUS_MAP = {
        PLC: "_hmi_status",
        ARM: "_arm_status",
        NAV: "_nav_status",
    }

    # AMCL Initialisation Constants
    GLOBAL_SEARCH_VARIANCE : float  = 999.0
    REQUIRED_AMCL_VARIANCE : float  = 0.1
    LOCALISATION_ATTEMPTS : int     = 20        # usual success within 5 - 10
    SERVICE_CALL_TIMEOUT : float    = 5.0

    ERROR_CLEAR_TIMEOUT : float     = 10.0      # error clear timeout
    MAX_ERRORS : int                = 3         # maximum allowable error attempts
    INITIALISE_TIME : float         = 12.0      # node initialisation timeout


    def __init__(self):
        """
        Initializes the Warehouse Coordinator node and its service clients.
        Sets up the main timer for the coordination logic.
        """
        super().__init__('warehouse_coordinator')
        self.get_logger().info('Initializing Warehouse Coordinator...')

        # Initialisation State
        self._initialised : bool            = False     # __init__ complete flag

        # Coordination State Variables
        self._hmi_status : str              = "idle"
        self._nav_status : str              = "idle"
        self._arm_status : str              = "idle"
        self._coord_state : int             = 0         # coordinator state. used for sequence and error management
        self._current_node : str | None     = None
        self._error_node : str | None       = None      # hold name of node with error
        self._error_counts : dict[str,tuple[int,Time | None]] = {         # dictionary of node error counts and error start times
            self.COR: (-1, None),
            self.PLC: (-1, None),
            self.ARM: (-1, None),
            self.NAV: (-1, None)
        }
        self._error_st : Time | None        = None      # error start time

        # Box Variables
        self._box_size : str | None         = None
        self._box_location : str | None     = None
        self._box_queue : dict | None       = None
        self._total_count : int             = 0

        # AMCL Initialisation Variance Variables
        self._amcl_x : float        = self.GLOBAL_SEARCH_VARIANCE
        self._amcl_y : float        = self.GLOBAL_SEARCH_VARIANCE
        self._amcl_yaw : float      = self.GLOBAL_SEARCH_VARIANCE
        self._amcl_start : bool     = False         # flag to start localisation
        self._amcl_complete : bool  = False         # state of initial localisation
        self._amcl_pose : Pose      = Pose(         # pose object to store current amcl pose estimate, default to 0,0,0
            position=Point(x=0.0, y=0.0, z=0.0), 
            orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
        )
        self._amcl_wait_begin : Time | None = None  # start time of wait for amcl

        # define locked thread for synchronous future responses
        self._goal_lock = threading.Lock()


        # ROS2 Topic Publisher
        self._coordinator_pub = self.create_publisher(          # dedicated coordinator topic
            String, 
            '/pnp/coordinator',
            10
        )

        self._initial_pose_pub = self.create_publisher(         # initial amcl pose topic
            PoseWithCovarianceStamped, 
            '/initialpose',
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

        self._amcl_pose_sub = self.create_subscription(         # amcl pose topic
            PoseWithCovarianceStamped,
            '/amcl_pose',
            self._handle_amcl_pose_update,
            10
        )
        self.get_logger().info('Now listening to AMCL Pose state...')

        # Service Clients for Costmap Clearing
        self._global_costmap_clear = self.create_client(        # global costmap
            ClearEntireCostmap,
            '/global_costmap/clear_entirely_global_costmap'
        )
        self._local_costmap_clear = self.create_client(         # local costmap
            ClearEntireCostmap,
            '/local_costmap/clear_entirely_local_costmap'
        )
        self.get_logger().info('Costmap clearing service clients created.')


        # Block Coordinator: pause and wait for all nodes to initialise
        time.sleep(self.INITIALISE_TIME)

        # Log Coordinator Started
        self.get_logger().info(f"📢 BEGIN: Coordinator Initialisation.")
        self.get_logger().info("Waiting for Global/Local Costmap Services...")

        # Check Costmap Services are Available
        global_ready : bool = self._global_costmap_clear.wait_for_service(timeout_sec=self.SERVICE_CALL_TIMEOUT)
        local_ready : bool  = self._local_costmap_clear.wait_for_service(timeout_sec=self.SERVICE_CALL_TIMEOUT)

        if not global_ready or not local_ready:
            # Skip Localisation if Costmap Services are Unavailable
            self.get_logger().fatal("Global/Local Costmap Clear service is not available after 5s.")
            self._amcl_start = self._amcl_complete = True
        else:
            self.get_logger().info("📍 Global/Local Costmap Clear service is available.")

        # Log Startup Complete
        self.get_logger().info("✅ Coordinator startup complete!")

        # Update Node Class Initialisation Complete
        self._initialised = True


    # --- Publishers ---
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


    # --- Callback Handles ---
    def _handle_status_update(self, msg : String) -> None:
        try:
            data = json.loads(msg.data)

            data_node = data.get("node")
            data_status = data.get("status")

            # reset current node (incase match fails)
            self._current_node = None

            # 1. Update internal status cache
            match data_node:
                case self.PLC:
                    self._hmi_status = data_status
                    self._handle_plc_status(data)
                    self._current_node = data_node
                case self.NAV:
                    self._nav_status = data_status
                    self._current_node = data_node
                case self.ARM:
                    self._arm_status = data_status
                    self._current_node = data_node
            
            # 2. Handle Error State
            if not self._error_check():
                return
            
            # 3. Skip Execution if a Node is Busy
            if self._nav_status == "busy" or self._arm_status == "busy" or self._hmi_status == "busy":
                return
            
            # 4. Wait for localisation
            if not self._amcl_start or not self._amcl_complete:
                if not self._amcl_wait_begin:
                    self._amcl_wait_begin = self.create_timer(
                        self.SERVICE_CALL_TIMEOUT,
                        self._amcl_complete_timer_callback
                    )
                return

            # 5. Run Sequence Step
            self._execute_coordinator()

        except json.JSONDecodeError as e:
            self.get_logger().error(f"❌ Failed to parse status update: {e}\nRaw msg={msg.data}")
            self._coord_state = 99
            self._error_check()

        except Exception as e:
            self.get_logger().error(f"❌ Failed to parse status update: {e}")
            self._coord_state = 99
            self._error_check()


    def _handle_plc_status(self, data) -> None:    
        # check for required data
        if 'current_target' in data:
            # get initial box data
            if self._coord_state == 0:
                self._box_size = data['current_target'].get('box_size', None)
                self._box_location = data['current_target'].get('location', None)
                self._total_count = data.get('total_count', 0)
                return

            # current and previous total box counts for comparison
            previous_count : int = self._total_count
            current_count : int = data.get('total_count', 0)

            # update box data on total count reset
            if current_count == 0:
                self._box_size = data['current_target'].get('box_size', None)
                self._box_location = data['current_target'].get('location', None)
                self._total_count = data.get('total_count', 0)

                self._box_queue = None
                self.get_logger().warn(f"Change Detected: Box count reset. Box queue cleared.")
                return
            

            # if status changes to idle with box in queue (box has been deleted)
            if self._hmi_status == "idle" and self._box_queue:
                self.get_logger().warn(f"Change Detected: Queued box deleted. Box queue cleared.")
                self._box_queue = None
                return

            # check if count has incremented (changed)
            if previous_count != current_count:
                # retrieve current box size and location from topic data
                box_size = data['current_target'].get('box_size', None)
                box_location = data['current_target'].get('location', None)
                
                # return if either are empty
                if not box_size or not box_location:
                    return

                # update box queue with latest box
                self._box_queue = {
                    "box_size": box_size,
                    "location": box_location,
                }
                self.get_logger().warn(f"📦 New box ({box_size}, {box_location}) queued. Current sequence in progress.")


    def _handle_amcl_pose_update(self, msg : PoseWithCovarianceStamped) -> None:
        if not self._amcl_complete:
            self._amcl_pose = msg.pose.pose
            self._amcl_x    = msg.pose.covariance[0]
            self._amcl_y    = msg.pose.covariance[7]
            self._amcl_yaw  = msg.pose.covariance[35]

            self.get_logger().info(f"AMCL Covariance: x={self._amcl_x:.2f}, y={self._amcl_y:.2f}, yaw={self._amcl_yaw}")     

        if not self._amcl_start and not self._amcl_complete:
            self._amcl_start = True             # update amcl localisation start flag
            self._initialisation_sequence()     # run initialisation sequence


    def _amcl_init_callback(self) -> None:
        # prevent callback while nav or arm is busy
        if self._nav_status == "busy" or self._arm_status == "busy":
            return
        
        # confidence achieved
        if self._amcl_confident():
            self._amcl_complete = True      # lock confidence

        # initialisation complete
        if self._amcl_complete:
            self.get_logger().info(f"AMCL confidence achieved on try {self._init_count}.")
            self.get_logger().info("✅ AMCL Initialisation Complete!")
            self._init_timer.cancel()                       # clear timer
            self._execute_new_thread(self._clear_costmaps)  # clear costmaps
            return
        
        # max localisation attempts reached
        if self._init_count >= self.LOCALISATION_ATTEMPTS:
            self.get_logger().error("❌ Failed to achieve required AMCL confidence after all attempts.")
            self._amcl_complete = True                      # end localisation attempts
            self._init_timer.cancel()                       # clear timer
            self._execute_new_thread(self._clear_costmaps)  # clear costmaps
            return
        
        # send initial pose with high covariance to begin localisation
        if self._init_count == 0:
            current_pose : Pose = Pose(
                position=Point(x=0.0, y=0.0, z=0.0), 
                orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
            )

            self.get_logger().warn("⚠️  Calling AMCL service to reinitialize global localisation...")
            self._execute_new_thread(self._clear_costmaps)  # clear costmaps
            self._reinitialize_amcl(                        # send new initial pose
                current_pose, 
                self.GLOBAL_SEARCH_VARIANCE, 
                self.GLOBAL_SEARCH_VARIANCE, 
                self.GLOBAL_SEARCH_VARIANCE
            )
            # spin robot
            self._publish_command(self.NAV, "_spin", 12.566)

            self._init_count = 1
            return
        
        # clear costmap before re-attempting localisation
        self._execute_new_thread(self._clear_costmaps)

        # periodically reset variance to a high value
        # stops localisation getting stuck in invalid area
        if self._init_count % 4 == 0:
            buffer : float = self._init_variance_buffer * (self._init_count / 2)
        else:
            buffer : float = self._init_variance_buffer / self._init_count

        self.get_logger().warn(f"AMCL Initialisation Try {self._init_count}: Re-seeding amcl pose with buffered covariance {buffer:.2f}.")

        # retreive current pose and covariance w/ buffer
        current_pose = self._amcl_pose
        x : float    = self._amcl_x + buffer
        y : float    = self._amcl_y + buffer
        yaw : float  = self._amcl_yaw + buffer

        # send new initial pose and spin robot
        self._reinitialize_amcl(current_pose, x, y, yaw)
        self._publish_command(self.NAV, "_spin", 12.566)

        # increment attempt count
        self._init_count += 1


    def _amcl_complete_timer_callback(self) -> None:
        """
        Called once after the post-localisation timeout has expired.
        Enables the coordinator to respond to external commands.
        """
        # skip while class is still initialising
        if not self._initialised:
            return

        # cancel timer on first call
        if self._amcl_wait_begin:
            self._amcl_wait_begin.cancel()

        # set initial amcl localisation to complete (skip localisation)
        if not self._amcl_start:
            self.get_logger().info(f"Skipping AMCL localisation initialisation sequence. Failed to begin localisation within {self.SERVICE_CALL_TIMEOUT}s.")
            self._amcl_start = self._amcl_complete = True

            # send single call to run coordinator sequence (runs any queued topic data)
            self._execute_coordinator()


    def _error_check(self) -> bool:
        # handle coordinator error
        if self._coord_state == 99:
            cor_errors, _ = self._error_counts.get(self.COR, (0, None))

            # update error counter from null state (-1)
            if cor_errors == -1:
                cor_errors = 0

            cor_errors += 1
            self._error_counts[self.COR] = (cor_errors, None)   # increment coordinator error count
            if cor_errors >= self.MAX_ERRORS:
                self.get_logger().error(f"FATAL ERROR: Maximum Fatal Error Count Reached. Please restart system.")
                rclpy.shutdown()
                return False
            
            # clear error state
            self.get_logger().info(f"Error has cleared! Resetting to IDLE.")
            self._coord_state = 0       # set coordinator state back to 'IDLE'

            return True


        all_errors_clear : bool = True     # flag for all errors cleared

        for node, (errors, st) in self._error_counts.items():
            # retrieve current (Node) status method name
            status_attr : str | None = self.NODE_STATUS_MAP.get(node, None)

            if status_attr:
                # retrieve current (Node) status
                node_status = getattr(self, status_attr)
                is_error : bool = (node_status == "error")

                # clear error count (if Node status isn't in error)
                if not is_error:
                    if errors >= 0:
                        self.get_logger().info(f"✅ ERROR CLEARED: Error from '{node}' cleared ({errors}/{self.MAX_ERRORS})")

                    self._error_counts[node] = (-1, None)                    
                    continue
                
                # error not cleared (update flag)
                all_errors_clear = False

                # update error counter from null state (-1)
                if errors == -1:
                    errors = 0

                # create error count increment timer (if not set)
                if not st:
                    self.get_logger().error(f"🚨 NEW ERROR DETECTED: Waitting for error to clear (node={node}, timeout={self.ERROR_CLEAR_TIMEOUT}s)...")
                    st = self.get_clock().now()     # update error timeout start time
                    self._error_counts[node] = (errors, st)

                # increment error count after timer elapse
                if (self.get_clock().now() - st) >= Duration(seconds=self.ERROR_CLEAR_TIMEOUT):
                    errors += 1                     # increment (Node) error count
                    st = self.get_clock().now()     # update error timeout start time
                    self._error_counts[node] = (errors, st)
                    self.get_logger().error(f"ERROR TIMEOUT: Error message from '{node}' not cleared ({errors}/{self.MAX_ERRORS}).")

                # shutdown coordinator if error limit exceeded
                if errors >= self.MAX_ERRORS:
                    self.get_logger().fatal(f"FATAL ERROR: Max errors received from '{node}' of {self.MAX_ERRORS}. Shutting down.")
                    rclpy.shutdown()


        return all_errors_clear


    # --- Localisation (AMCL) ---
    def _initialisation_sequence(self) -> None:
        # localisation is complete or required confidence is met, skip relocalisation
        if self._amcl_complete:
            self.get_logger().warn("Skipping AMCL localisation initialisation sequence...")
            return

        self.get_logger().info("Running Initialisation Sequence...")

        # Ensure Arm is in Safe Moving Position
        self._publish_command(self.ARM, "_arm_move", "moving")

        # Spin Robot to Update Sensor Position Data
        self._publish_command(self.NAV, "_spin", 12.566)

        self._init_count : int = 0
        self._init_variance_buffer : float = 50.0

        self._init_timer = self.create_timer(
            self.LOCALISATION_ATTEMPTS,
            self._amcl_init_callback
        )


    def _reinitialize_amcl(self, pose : Pose, x : float, y : float, yaw : float) -> None:
        """
        Calls the AMCL service to reinitialize global localization.
        This forces AMCL to spread particles across the entire map.
        """
        self.get_logger().warn(
            f"Publishing initial pose at ({pose.position.x:.2f}, {pose.position.y:.2f}) "
            f"with variance (x={x:.2f}, y={y:.2f}, yaw={yaw:.2f})"
        )

        pose_msg = PoseWithCovarianceStamped()
        pose_msg.header.stamp = Time().to_msg()
        pose_msg.header.frame_id = 'map'

        # 1. Set Position
        pose_msg.pose.pose = pose

        pose_msg.pose.covariance = [
            x, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, y, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, yaw    # (index 35)
        ]

        self._initial_pose_pub.publish(pose_msg)


    def _amcl_confident(self) -> bool:
        if (self._amcl_x < self.REQUIRED_AMCL_VARIANCE and self._amcl_y < self.REQUIRED_AMCL_VARIANCE and self._amcl_yaw < self.REQUIRED_AMCL_VARIANCE):
            return True
        return False


    # --- Costmap Clearer ---
    def _clear_costmaps(self, map_type : str = "both") -> bool:
        """
        Calls the Nav2 service to clear the global and/or local costmaps.

        Args:
            map_type (str): 'global', 'local', or 'both' (default).
        
        Returns:
            bool: True if all requested service calls were successful, False otherwise.
        """
        request = ClearEntireCostmap.Request()  # Clear request
        success = True                          # Success Flag

        if map_type in ('global', 'both'):
            self.get_logger().warn("Clearing global costmap...")

            future = self._global_costmap_clear.call_async(request)

            # get curent time a duration time
            st = self.get_clock().now()
            td = Duration(seconds=self.SERVICE_CALL_TIMEOUT)

            # wait 'SERVICE_CALL_TIMEOUT' seconds for future result
            while (self.get_clock().now() - st) <= td:
                if future.done():
                    break
                time.sleep(0.05)

            if future.done() and future.result() is not None:
                self.get_logger().info("Global costmap successfully cleared.")
            else:
                self.get_logger().error("Failed to clear global costmap (timed out or service failed).")
                success = False
        
        if map_type in ('local', 'both'):
            self.get_logger().warn("Clearing local costmap...")

            future = self._local_costmap_clear.call_async(request)

            # get curent time a duration time
            st = self.get_clock().now()
            td = Duration(seconds=self.SERVICE_CALL_TIMEOUT)

            # wait 'SERVICE_CALL_TIMEOUT' seconds for future result
            while (self.get_clock().now() - st) <= td:
                if future.done():
                    break
                time.sleep(0.05)

            if future.done() and future.result() is not None:
                self.get_logger().info("Local costmap successfully cleared.")
            else:
                self.get_logger().error("Failed to clear local costmap (timed out or service failed).")
                success = False

        self._execute_coordinator()
        return success


    # --- Threading Method ---
    def _execute_new_thread(self, method, value = None) -> None:
        # the thread lock is already held
        if not self._goal_lock.acquire(blocking=False):
            self.get_logger().warn("Thread Lock Busy: Skipping execution.")
            return
        
        def thread_wrapper():
            try:
                if value is not None and value != "":
                    method(value)
                else:
                    method()

            except Exception as e:
                self.get_logger().error(f"Error during threaded execution of {method.__name__}: {e}")
            
            finally:
                self.get_logger().info(f"Execution thread released (complete).")
                self._goal_lock.release()

        try:
            t = threading.Thread(target=thread_wrapper, daemon=True)
            t.start()
            self.get_logger().info(f"New execution thread started...")

        except Exception as e:
            self.get_logger().error(f"Error during threaded execution of {method.__name__}: {e}")



    # --- Main Logic Loop ---
    def _execute_coordinator(self) -> None:
        # -- Start Sequence ---

        # State 0: 
        if self.validate_status(0, nav="idle", arm="idle", hmi="ready"):
            self.get_logger().info("Starting Coordination Sequence...")

            self._publish_command(self.ARM, "_set_wait_seconds", 3.0)
            self._publish_command(self.NAV, "_set_wait_seconds", 3.0)

            self.get_logger().info(f"STATUS READY: Box '{self._box_size}' at '{self._box_location}' detected. Starting sequence.")
            self._coord_state = 1

            time.sleep(0.5)
            self._execute_coordinator()
            return

        # State 1:
        if self.validate_status(1, arm="idle", hmi="ready"):
            self._publish_command(self.ARM, "_arm_move", "moving")
            self._coord_state = 2
            return

        # State 2:
        if self.validate_status(2, nav="idle", arm="complete"):
            self._publish_command(self.NAV, "_go_to_waypoint", "conveyor")
            self._coord_state = 3
            return

        # State 3:
        if self.validate_status(3, nav="complete", arm="idle", hmi="ready"):
            self._publish_command(self.ARM, "_arm_pick_box", self._box_size)
            self._coord_state = 4
            return
        # handle box deleted or conveyor error
        elif self.validate_status(3, nav="complete", hmi="idle"):
            self._publish_command(self.NAV, "_go_to_waypoint", "home")
            self._coord_state = 0
            return

        # State 4:
        if self.validate_status(4, arm="idle"):
            self._publish_command(self.ARM, "_arm_move", "carry_box")
            self._coord_state = 5
            return

        # State 5:
        if self.validate_status(5, nav="idle", arm="idle"):
            self._publish_command(self.NAV, "_go_to_waypoint", "drop-off")
            self._coord_state = 6
            return

        # State 6:
        if self.validate_status(6, nav="complete", arm="idle"):
            self._publish_command(self.ARM, "_arm_move", "place_box")
            self._coord_state = 7
            return

        # State 7:
        if self.validate_status(7, nav="idle", arm="complete"):
            self._publish_command(self.ARM, "_arm_move", "moving")
            self._coord_state = 8
            return

        # Interupt: Begin Queued Box Sequence (at safe intermediate state: return to home)
        if self._coord_state >= 8 and self._nav_status in ("idle", "complete") and self._arm_status in ("idle", "complete"):
            if self._box_queue:
                self._run_queued_box()
                return

        # State 8:
        if self.validate_status(8, nav="idle", arm="complete"):
            self._publish_command(self.NAV, "_go_to_waypoint", "home")
            self._coord_state = 9
            return

        # State 9:
        if self.validate_status(9, nav="complete", arm="idle"):
            self._publish_command(self.ARM, "_arm_move", "home")
            self._coord_state = 10
            return

        # State 10:
        if self.validate_status(10, nav="idle", arm="complete"):
            self.get_logger().info("✅ Full pick, nav, and place sequence successfully completed and arm is home! Back to IDLE.")
            self._coord_state = 0
            # clear costmap of dynamic objects (incase position has moved)
            self._execute_new_thread(self._clear_costmaps)
        
        return


    def validate_status(self, coord_state : int, nav : str = "", arm : str = "", hmi : str = "") -> bool:
        if coord_state != self._coord_state:
            return False
        
        if nav and self._nav_status != nav:
            return False

        if arm and self._arm_status != arm:
            return False

        if hmi and self._hmi_status != hmi:
            return False
        
        self.get_logger().info(f"CURRENT EXECUTION STEP: {self.COORD_STATES.get(self._coord_state)}")

        return True


    def _run_queued_box(self) -> None:
        self.get_logger().warn("📦 Exiting sequence early and running queued box.")
        
        self._box_size = self._box_queue['box_size']
        self._box_location = self._box_queue['location']

        self._box_queue = None

        self.get_logger().info("✅ Pick, nav, and place sequence successfully completed.")
        self._coord_state = 0

        self._execute_coordinator()



def main(args=None):

    rclpy.init(args=args)
    node = WarehouseCoordinator()

    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        node.get_logger().info('Starting coordination executor. All nodes spun up.')
        executor.spin()
    except KeyboardInterrupt:
        node.info('Coordinator stopped by user (KeyboardInterrupt).')
    finally:
        node.get_logger().info('Coordinator shutting down.')
        node.destroy_node()
        rclpy.shutdown()