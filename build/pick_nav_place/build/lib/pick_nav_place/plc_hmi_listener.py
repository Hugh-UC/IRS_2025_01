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
        self._box_size : str | None                 = None
        self._box_location : str | None             = None

        self._previous_box_size : str | None        = None

        self._status : int = 0
        self._handle_error_time : float | None  = None
        self._has_published : bool              = False

        
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



    # --- Callback Method ---
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

            self._box_size : str | None     = self._get_box_weight()
            self._box_location : str | None = self._get_box_location()

            self._log_data()        # log data


            if not self._check_status():
                # error logging handled by _check_status
                return
            

            if self._detect_box_update():
                match self._status:
                    case 0:
                        self.get_logger().info(f"Published new PLC Status to {self._hmi_status_pub.topic_name}. New task ready!")
                        # set and publish 'busy' status
                        self._status = 1
                        self._publish_status()

                        # wait for box position to finalise
                        self._wait_box_ready()
                    
                    case 1:
                        self.get_logger().warn(f"Change Detected: Box has been removed from the conveyor before reaching the correct location.")

                        # clear box waiting
                        self._stop_poll_timer()

                        self._status = 0
                        self._publish_status()

                    case 2:
                        if not self._box_size and not self._box_location:
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
                "box_size": self._get_box_key(),
                "location": self._box_location
            },
            "total_count": self._get_total_count(),
            "status": self._get_status_name(),       # "idle", "busy", "ready", "error"
            "timestamp": int(self.get_clock().now().nanoseconds / 1e9), # Unix timestamp
        }

        json_payload : json = json.dumps(payload)

        msg : String = String()
        msg.data = json_payload

        self._hmi_status_pub.publish(msg)


    def _get_status_name(self) -> str:
        """
        Takes the current status of the robot arm and returns its translated string representation.

        Returns:
            str: string of status name
        """
        return self.STATUS_LABELS[self._status]


    def _check_status(self) -> bool:
        """
        _summary_

        Returns:
            bool: _description_
        """
        if self._status == 3:
            self._handle_status_error()
            return False
        
        if self._status == 1:
            if not self._has_published:
                self._has_published = True
                self.get_logger().warn("Warehouse Conveyor is 'busy'.")

            return True
        
        self._has_published = False

        return True


    def _handle_status_error(self) -> None:
        """
        _summary_
        """
        # get current time
        current_time : float = self.get_clock().now().nanoseconds / 1e9

        # begin error timer
        if not self._handle_error_time:
            self.get_logger().error(f"Error with Warehouse Conveyor.")
            self.get_logger().error(f"Please remove boxes to clear error!")

            # set error timer
            self._handle_error_time : float = self.get_clock().now().nanoseconds / 1e9

            return

        # check box cleared, reset status
        if not self._box_location and not self._box_size:
            self._handle_error_time = None      # reset error timer

            # publish idle status 
            self.get_logger().info(f"Error has been successfully cleared.")
            self._status = 0
            self._publish_status()

            return

        # handle timer expire
        if self._handle_error_time:
            if current_time - self._handle_error_time > self.MAX_WAIT_TIME:
                self._handle_error_time = None      # reset error timer

                # publish second error status to coordinator (notifies coordinator node is no longer responsive).
                self.get_logger().error(f"FATAL ERROR: Error failed to clear within max wait time '{self.MAX_WAIT_TIME}' seconds.")
                self._status = 3
                self._publish_status()
        
        return



    # --- Box Poll/Detectors
    def _detect_box_update(self) -> bool:
        # match previous with new box weight
        if self._box_size != self._previous_box_size:
            self.get_logger().info(f"Change Detected: Current Weight = {self._box_size}, Previous_Weight = {self._previous_box_size}")
            # update previous box weight
            self._previous_box_size = self._box_size

            return True

        return False


    def _wait_box_ready(self) -> None:
        self._expected : tuple[str | None, str | None, float | None] = None, None, None

        if not self._box_size:
            self.get_logger().error("Box weight was not available. Cannot determine expected location.")
            self._status = 3
            self._publish_status()
            return

        expected_location = self._get_expected_location()

        if not expected_location:
            self.get_logger().error("Failed to map box weight to a known size key and expected location.")
            self._status = 3
            self._publish_status()
            return
        
        self._expected = expected_location, self.get_clock().now().nanoseconds / 1e9
        
        self.get_logger().info(f"Waiting for box location to be finalised (Expected: '{expected_location}') with a {self.MAX_WAIT_TIME} second timeout...")
        
        # create box location timer
        self._status_check_timer = self.create_timer(self.POLL_FREQUENCY, self._poll_box_ready)


    def _poll_box_ready(self) -> None:
        current_time : float = self.get_clock().now().nanoseconds / 1e9
        current_box_location : str | None = self._get_box_location()

        expected_location, start_time = self._expected

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
                # Update box location
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
    def _get_box_key(self) -> str | None:
        """
        Parses the weight string (e.g., '6 kg') to determine the box size key ('small', 'medium', 'big').
        """
        if not self._box_size:
            return None

        try:
            # 1. Extract the number from the string
            weight_val : float = float(self._box_size.lower().replace(' kg', '').strip())
        except ValueError:
            self.get_logger().error(f"Failed to parse weight string: {self._box_size}.")
            return None
        
        # 2. Map weight to size key (Assumed thresholds: <10=Small, <20=Medium, >=20=Big)
        if weight_val <= 5.0:
            return "small"
        elif weight_val <= 10.0:
            return "medium"
        elif weight_val > 10.0:
            return "big"
        
        self.get_logger().error(f"Weight {weight_val} kg does not match a known box size range.")
        return None


    def _get_expected_location(self) -> str | None:
        """
        Parses the box_key string (e.g. 'small') to determine the expected location ('A', 'B', 'C').
        """
        box_key = self._get_box_key()
        
        # 2. Map weight to size key (Assumed thresholds: <10=Small, <20=Medium, >=20=Big)
        if box_key == "small":
            return "C"
        elif box_key == "medium":
            return "B"
        elif box_key == "big":
            return "A"
        
        self.get_logger().error(f"Box key '{box_key}' does not match a known box size range.")
        return None




    def _get_box_weight(self) -> str | None:
        """
        Retrieves the detected box weight as a string.

        Returns:
            str | None: weight string (e.g., '6 kg', '14 kg'), or None.
        """
        if self._box_data:

            weight : str | None = self._box_data.get('weight_raw')

            weight = weight.replace("\n", "").replace(": ", "").strip()

            if not weight or weight == "" or weight == "0 kg":
                return None

            return str(weight)
        
        return None


    def _get_box_location(self) -> str | None:
        """
        Retrieves the detected box location as a string.

        Returns:
            str | None: location string (e.g., 'A', 'B', or 'C'), or None.
        """
        if self._box_data:
            location : str | None = self._box_data.get('location')

            location = location.replace("\n", "").replace(": ", "").replace("-", "").strip()

            if not location or location == "":
                return None

            return str(location)

        return None



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