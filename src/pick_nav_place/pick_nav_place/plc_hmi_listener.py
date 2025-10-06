import datetime
from datetime import timezone

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
from typing import Any

class WarehouseHmiListener(Node):
    """
    ROS 2 Node responsible for listening to the 'hmi/unified_status' topic.
    Receiving real-time status updates from the warehouse environment, including box detection data and cumulative counts.
    It parses the JSON string message into internal attributes for easy access by other nodes.

    Args:
        Node (rclpy.node): node class from ROS 2 client library,
                           core node functionality is inherited from class.
    """
    def __init__(self):
        """
        Initializes internal attributes to store the latest parsed data 
        and creates a subscription to the 'hmi/unified_status' topic.
        """
        super().__init__('warehouse_hmi_listener')

        self._stamp : dict[str, int] | None         = None
        self._box_data : dict[str, Any] | None      = None
        self._box_counts : dict[str, int] | None    = None
        self._raw_data : dict[str, Any] | None      = None

        self.subscription = self.create_subscription(
            String,
            'hmi/unified_status', # listening to the ros2 PLC topic
            self._listener_callback,
            10
        )
        
        self.get_logger().info('HMI/PLC Listener node started.')


    def _listener_callback(self, msg : str) -> None:
        """
        Callback function executed on receiving a new message from the topic.

        Args:
            msg (str): incoming message containing the JSON data string.
        """
        try:
            data = json.loads(msg.data)

            self._stamp = data.get("stamp", None)
            self._box_data = data.get("box", None)
            self._box_counts = data.get("counts", None)
            self._raw_data = data

            self._log_data()
        
        except Exception as e:
            self.get_logger().error(f"Failed to parse JSON: {e}\nRaw msg={msg.data}")


    def _log_data(self) -> None:
        """
        Logs received HMI/PLC data in human readable format in debug.
        """
        self.get_logger().debug("📥📥 Received PLC status:")
        self.get_logger().debug(f" ⏱⏱ Time: {self.get_pretty_time()}")
        self.get_logger().debug(f" 📦📦 Box weight raw = {self.get_box_weight()}")
        self.get_logger().debug(f" 📍📍 Location: {self.get_location()}")
        self.get_logger().debug(f" 🔢🔢 Counts: big={self.get_big_count()}, medium={self.get_medium_count()}, "
                                f"small={self.get_small_count()}, total={self.get_total_count()} \n")


    # --- Getters ---
    def get_stamp(self) -> dict | None:
        """
        Retrieves the last received timestamp data from the HMI/PLC.

        Returns:
            dict | None: dictionary containing 'sec' and 'nanosec', or None.
        """
        return self._stamp


    def get_stamp_sec(self) -> int | None:
        """
        Retrieves the last received timestamp in seconds from the HMI/PLC as integer.

        Returns:
            int | None: timestamp seconds integer, or None.
        """
        if self._stamp:
            sec = self._stamp.get('sec')
            return int(sec) if sec is not None else None
        
        return None
    

    def get_stamp_nanosec(self) -> int | None:
        """
        Retrieves the last received timestamp in nano seconds from the HMI/PLC as integer.

        Returns:
            int | None: timestamp nano seconds integer, or None.
        """
        if self._stamp:
            nanosec = self._stamp.get('nanosec')
            return int(nanosec) if nanosec is not None else None
        
        return None


    def get_pretty_time(self) -> str | None:
        """
        Converts the ROS 2 timestamp (sec/nanosec) into a human-readable UTC string.

        Returns:
            str | None: formatted time string (e.g., '01/01/2025 - 00:00:00.000000 AM UTC'), or None.
        """
        sec : int       = self.get_stamp_sec()
        nanosec : int   = self.get_stamp_nanosec()

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


    def get_box_data(self) -> dict | None:
        """
        Retrieves the last received box-specific data (e.g., location, weight).

        Returns:
            dict | None: dictionary containing box details, or None.
        """
        return self._box_data


    def get_box_weight(self) -> str | None:
        """
        Retrieves the detected box weight as a string.

        Returns:
            str | None: weight string (e.g., '6 kg', '14 kg'), or None.
        """
        if self._box_data:
            weight = self._box_data.get('weight_raw')
            return str(weight) if weight is not None else None
        
        return None


    def get_counts(self) -> dict | None:
        """
        Retrieves the last received cumulative box counts (small, medium, big, total).

        Returns:
            dict | None: dictionary containing box counts, or None.
        """
        return self._box_counts


    def get_total_count(self) -> int:
        """
        Retrieves the total box count as a integer.

        Returns:
            int | None: total box count integer, or None.
        """
        if self._box_counts:
            total = self._box_counts.get('total')
            return int(total) if total is not None else None

        return None


    def get_big_count(self) -> int:
        """
        Retrieves the big box count as a integer.

        Returns:
            int | None: big box count integer, or None.
        """
        if self._box_counts:
            big = self._box_counts.get('big')
            return int(big) if big is not None else None

        return None


    def get_medium_count(self) -> int:
        """
        Retrieves the medium box count as a integer.

        Returns:
            int | None: medium box count integer, or None.
        """
        if self._box_counts:
            medium = self._box_counts.get('medium')
            return int(medium) if medium is not None else None

        return None


    def get_small_count(self) -> int:
        """
        Retrieves the small box count as a integer.

        Returns:
            int | None: small box count integer, or None.
        """
        if self._box_counts:
            small = self._box_counts.get('small')
            return int(small) if small is not None else None

        return None


    def get_location(self) -> str | None:
        """
        Retrieves the detected box location as a string.

        Returns:
            str | None: location string (e.g., 'A', 'B', or 'C'), or None.
        """
        if self._box_data:
            location = self._box_data.get('location')
            return str(location) if location is not None else None

        return None


    def get_raw_json_data(self) -> dict | None:
        """
        Retrieves the last full dictionary parsed from the HMI/PLC message.

        Returns:
            dict | None: full parsed dictionary, or None.
        """
        return self._raw_data



def main(args=None) -> None:
    """
    Initializes the ROS 2 node and spins the WarehouseHmiListener.
    """
    rclpy.init(args=args)
    warehouse_hmi_listener = WarehouseHmiListener()
    try:
        rclpy.spin(warehouse_hmi_listener)
    except KeyboardInterrupt:
        warehouse_hmi_listener.get_logger().info('HMI Listener stopped by user.')
    finally:
        warehouse_hmi_listener.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()