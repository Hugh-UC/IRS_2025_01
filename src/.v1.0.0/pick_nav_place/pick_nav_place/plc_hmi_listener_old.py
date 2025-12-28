#!/usr/bin/env python3
import time
import datetime
from datetime import timezone

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
from typing import Any

class PLCHmiListener(Node):
    """
    ROS 2 Node responsible for listening to the 'hmi/unified_status' topic.
    Receiving real-time status updates from the warehouse environment, including box detection data and cumulative counts.
    It parses the JSON string message into internal attributes for easy access by other nodes.

    Args:
        Node (rclpy.node): node class from ROS 2 client library,
                           core node functionality is inherited from class.
    """
    STATUS_LABELS : list[str] = ["idle", "busy", "ready", "error"]
    MAX_WAIT_TIME : float   = 6.0
    POLL_FREQUENCY : float  = 0.1   # 100ms

    def __init__(self):
        """
        Initializes internal attributes to store the latest parsed data 
        and creates a subscription to the 'hmi/unified_status' topic.
        """
        super().__init__('plc_hmi_listener')

        # variables to store json data from /hmi/unified_status
        self._stamp : dict[str, int] | None         = None
        self._box_data : dict[str, Any] | None      = None
        self._box_counts : dict[str, int] | None    = None
        self._raw_data : dict[str, Any] | None      = None

        self._box_size : str                        = "Null"
        self._box_location : str                    = "Null"
        self._previous_weight : str | None          = self._get_box_weight()

        self._status : int = 0

        self.handle_error_time : float | None = None


        
        self._hmi_status_sub = self.create_subscription(
            String,
            'hmi/unified_status', # listening to the ros2 PLC topic
            self._listener_callback,
            10
        )

        # initialise waypoint status publisher
        self._hmi_status_pub = self.create_publisher(
            String, 
            '/pnp/hmi_status',  # dedicated topic name
            10
        )
        
        self.get_logger().info('HMI/PLC Listener node started.')
        self._publish_status()



    def _listener_callback(self, msg : str) -> None:
        """
        Callback function executed on receiving a new message from the topic.

        Args:
            msg (str): incoming message containing the JSON data string.
        """
        try:
            # load json data
            data = json.loads(msg.data)

            self._stamp = data.get("stamp", None)
            self._box_data = data.get("box", None)
            self._box_counts = data.get("counts", None)
            self._raw_data = data

            # log data
            self._log_data()

            if not self._check_status():
                self._handle_status_error()
                return

            if self._detect_box_update():
                if self._status == 0:
                    self.get_logger().info(f"Published new PLC Status to {self._hmi_status_pub.topic_name}. New task ready!")
                    # set and publish 'busy' status
                    self._status = 1
                    self._publish_status()

                    self._wait_box_ready()

                if self._status == 2:
                    self._box_size = "Null"
                    self._box_location = "Null"

                    self._status = 0
                    self._publish_status()

            
        except Exception as e:
            self.get_logger().error(f"Failed to parse JSON: {e}\nRaw msg={msg.data}")



    # --- Status Managers ---
    def _publish_status(self) -> None:
        # create message payload
        payload : dict[str, str | int] = {
            "node": self.get_name(),                # "plc_hmi_listener"
            "current_target": {
                "box_size": self._box_size,
                "location": self._box_location
            },
            "status": self._get_status_name(),       # "idle", "busy", "ready", "error"
            "timestamp": int(self.get_clock().now().nanoseconds / 1e9), # Unix timestamp
        }

        json_payload : json = json.dumps(payload)

        msg : String = String()
        msg.data = json_payload

        self._hmi_status_pub.publish(msg)


    def _check_status(self) -> bool:
        """
        _summary_

        Returns:
            bool: _description_
        """
        if self._status == 1:
            # self.get_logger().info("Warehouse Conveyor is 'busy'. Ignoring new command(s).")
            return False
        
        if self._status == 3:
            return False
        
        return True


    def _handle_status_error(self) -> None:
        if self._status == 3:
            current_time : float = self.get_clock().now().nanoseconds / 1e9

            if not self.handle_error_time:
                self.get_logger().error("Error with Warehouse Conveyor. Ignoring new command(s).")
                self.get_logger().error(f"Please clear boxes to remove error!")
                self.handle_error_time : float = self.get_clock().now().nanoseconds / 1e9

            if current_time - self.handle_error_time > self.MAX_WAIT_TIME:
                self.handle_error_time = None

                if not self._get_box_location() and self._get_box_weight() == "0 kg":
                    self._box_size = "Null"
                    self._box_location = "Null"

                    self.get_logger().info("Error has been successfully cleared. Now accepting new command(s).")
                    self._status = 0
                    self._publish_status()
        
        return


    def _get_status_name(self) -> str:
        """
        Takes the current status of the robot arm and returns its translated string representation.

        Returns:
            str: string of status name
        """
        return self.STATUS_LABELS[self._status]



    # --- Box Poll/Detectors
    def _detect_box_update(self) -> bool:
        # get current data
        current_location: str | None = self._get_box_location()
        current_weight: str | None = self._get_box_weight()

        if not current_weight:
            self.get_logger().debug("Incomplete or null total box weight received, ignoring update.")
            return False
        
        if self._status == 2 and current_weight == "0 kg":
            self.get_logger().info(f"Change Detected: Box has been removed from the conveyor.")
            return True

        # compare and return result
        if current_weight != self._previous_weight and current_weight != "0 kg":
            self.get_logger().info(f"Current Weight = {current_weight}, Previous_Weight = {self._previous_weight}")
            self._previous_weight = current_weight
            self.get_logger().info(f"New Task Detected: Location/Weight change (L:{current_location}, W:{current_weight}).")
            return True

        return False



    def _wait_box_ready(self) -> None:
        # retrieve box weight and determine expected location
        box_weight: str | None = self._get_box_weight()

        if not box_weight:
            self.get_logger().error("Box weight was not available. Cannot determine expected location.")
            self._status = 3
            self._publish_status()
            return

        box_size_key, expected_location = self._get_expected_box(box_weight)

        if not box_size_key or not expected_location:
            self.get_logger().error("Failed to map box weight to a known size key and expected location.")
            self._status = 3
            self._publish_status()
            return
        
        self._expected : tuple[str, str, float] = box_size_key, expected_location, self.get_clock().now().nanoseconds / 1e9
        
        self.get_logger().info(f"Waiting for box location to be finalised (Expected: '{expected_location}') with a {self.MAX_WAIT_TIME} second timeout...")
        
        # create box location timer
        self._status_check_timer = self.create_timer(self.POLL_FREQUENCY, self._poll_box_ready)



    def _poll_box_ready(self) -> None:
        current_time : float = self.get_clock().now().nanoseconds / 1e9
        current_box_location : str | None = self._get_box_location()

        box_size_key, expected_location, start_time = self._expected

        if current_time - start_time > self.MAX_WAIT_TIME:
            self._stop_poll_timer()

            self.get_logger().error(f"Timed out after {self.MAX_WAIT_TIME} seconds waiting for final box location.")
            self._status = 3
            self._publish_status()

            return


        if current_box_location:
            if expected_location == current_box_location:
                self._stop_poll_timer()

                # SUCCESS: Return True, size, and confirmed location
                self.get_logger().info(f"Box location '{current_box_location}' confirmed and matches expected location '{expected_location}'.")
                self._box_size = box_size_key
                self._box_location = current_box_location

                # Ready for pickup
                self._status = 2
                self._publish_status()

                return
            
            elif current_time - start_time >= self.MAX_WAIT_TIME + self.POLL_FREQUENCY:
                self.get_logger().error(f"Box location finalised to '{current_box_location}', but expected '{expected_location}'. Aborting pick.")
                return
            

    def _stop_poll_timer(self) -> None:
        """Cleans up the timer and state variables after success or failure."""
        if self._status_check_timer:
            self._status_check_timer.cancel()
            self._status_check_timer = None
        
        self.expected = None, None, 0.0




    # --- Pretty Data Loggers ---
    def _log_data(self) -> None:
        """
        Logs received HMI/PLC data in human readable format in debug.
        """
        self.get_logger().debug("📥📥 Received PLC status:")
        self.get_logger().debug(f" ⏱⏱ Time: {self._get_pretty_time()}")
        self.get_logger().debug(f" 📦📦 Box weight raw = {self._get_box_weight()}")
        self.get_logger().debug(f" 📍📍 Location: {self._get_box_location()}")
        self.get_logger().debug(f" 🔢🔢 Counts: big={self._get_big_count()}, medium={self._get_medium_count()}, "
                                f"small={self._get_small_count()}, total={self._get_total_count()} \n")



    # --- Getters ---
    def _get_expected_box(self, weight_str: str) -> tuple[str | None, str | None]:
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


    def _get_stamp(self) -> dict | None:
        """
        Retrieves the last received timestamp data from the HMI/PLC.

        Returns:
            dict | None: dictionary containing 'sec' and 'nanosec', or None.
        """
        return self._stamp


    def _get_stamp_sec(self) -> int | None:
        """
        Retrieves the last received timestamp in seconds from the HMI/PLC as integer.

        Returns:
            int | None: timestamp seconds integer, or None.
        """
        if self._stamp:
            sec = self._stamp.get('sec')
            return int(sec) if sec is not None else None
        
        return None
    

    def _get_stamp_nanosec(self) -> int | None:
        """
        Retrieves the last received timestamp in nano seconds from the HMI/PLC as integer.

        Returns:
            int | None: timestamp nano seconds integer, or None.
        """
        if self._stamp:
            nanosec = self._stamp.get('nanosec')
            return int(nanosec) if nanosec is not None else None
        
        return None


    def _get_pretty_time(self) -> str | None:
        """
        Converts the ROS 2 timestamp (sec/nanosec) into a human-readable UTC string.

        Returns:
            str | None: formatted time string (e.g., '01/01/2025 - 00:00:00.000000 AM UTC'), or None.
        """
        sec : int       = self._get_stamp_sec()
        nanosec : int   = self._get_stamp_nanosec()

        if sec is None or nanosec is None:
            return None
        
        timestamp = float(sec) + (nanosec / 1_000_000_000.0)

        # convert to a UTC datetime object
        dt_object_utc = datetime.datetime.fromtimestamp(timestamp, timezone.utc)

        date_time = dt_object_utc.strftime('%d/%m/%Y - %I:%M:%S')
        microsec_part = dt_object_utc.strftime('%f')
        am_pm_part = dt_object_utc.strftime('%p')

        pretty_time = f"{date_time}.{microsec_part} {am_pm_part} UTC"
        
        return pretty_time


    def _get_box_data(self) -> dict | None:
        """
        Retrieves the last received box-specific data (e.g., location, weight).

        Returns:
            dict | None: dictionary containing box details, or None.
        """
        return self._box_data


    def _get_box_weight(self) -> str | None:
        """
        Retrieves the detected box weight as a string.

        Returns:
            str | None: weight string (e.g., '6 kg', '14 kg'), or None.
        """
        if self._box_data:

            weight : str | None = self._box_data.get('weight_raw')

            if not weight or weight == "":
                return None

            weight = weight.replace("\n", "").replace(": ", "").strip()

            return str(weight)
        
        return None


    def _get_counts(self) -> dict | None:
        """
        Retrieves the last received cumulative box counts (small, medium, big, total).

        Returns:
            dict | None: dictionary containing box counts, or None.
        """
        return self._box_counts


    def _get_total_count(self) -> int:
        """
        Retrieves the total box count as a integer.

        Returns:
            int | None: total box count integer, or None.
        """
        if self._box_counts:
            total = self._box_counts.get('total')
            return int(total) if total is not None else None

        return None


    def _get_big_count(self) -> int:
        """
        Retrieves the big box count as a integer.

        Returns:
            int | None: big box count integer, or None.
        """
        if self._box_counts:
            big = self._box_counts.get('big')
            return int(big) if big is not None else None

        return None


    def _get_medium_count(self) -> int:
        """
        Retrieves the medium box count as a integer.

        Returns:
            int | None: medium box count integer, or None.
        """
        if self._box_counts:
            medium = self._box_counts.get('medium')
            return int(medium) if medium is not None else None

        return None


    def _get_small_count(self) -> int:
        """
        Retrieves the small box count as a integer.

        Returns:
            int | None: small box count integer, or None.
        """
        if self._box_counts:
            small = self._box_counts.get('small')
            return int(small) if small is not None else None

        return None


    def _get_box_location(self) -> str | None:
        """
        Retrieves the detected box location as a string.

        Returns:
            str | None: location string (e.g., 'A', 'B', or 'C'), or None.
        """
        if self._box_data:
            location : str | None = self._box_data.get('location')

            if not location or location == "":
                return None
            
            location = location.replace("\n", "").replace(": ", "").replace("-", "").strip()

            return str(location)

        return None


    def _get_raw_json_data(self) -> dict | None:
        """
        Retrieves the last full dictionary parsed from the HMI/PLC message.

        Returns:
            dict | None: full parsed dictionary, or None.
        """
        return self._raw_data



def main(args=None) -> None:
    """
    Initializes the ROS 2 node and spins the PLCHmiListener.
    """
    rclpy.init(args=args)
    node = PLCHmiListener()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('HMI Listener stopped by user.')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()