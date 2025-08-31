import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json

class WarehouseHmiListener(Node):
    def __init__(self):
        super().__init__('warehouse_hmi_listener')
        self.subscription = self.create_subscription(
            String,
            'hmi/unified_status', # listening to the ros2 PLC topic
            self.listener_callback,
            10)

    def listener_callback(self, msg):
        try:
            data = json.loads(msg.data)
            stamp = data["stamp"]
            box = data["box"]
            counts = data["counts"]

            print("📥📥 Received PLC status:")
            print(f" ⏱ Time: {stamp['sec']}.{stamp['nanosec']}")
            print(f" 📦📦 Box weight raw={box['weight_raw']}")
            print(f" 📍📍 Location: {box['location']}")
            print(f" 🔢🔢 Counts: big={counts['big']}, medium={counts['medium']}, "
                  f"small={counts['small']}, total={counts['total']}")
            print()
        except Exception as e:
            self.get_logger().error(f"Failed to parse JSON: {e}\nRaw msg={msg.data}")

def main(args=None):
    rclpy.init(args=args)
    warehouse_hmi_listener = WarehouseHmiListener()
    rclpy.spin(warehouse_hmi_listener)
    warehouse_hmi_listener.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
