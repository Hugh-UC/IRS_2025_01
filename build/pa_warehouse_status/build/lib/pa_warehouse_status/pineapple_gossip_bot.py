import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class PineappleGossipBot(Node):

    def __init__(self):
        super().__init__('pineapple_gossip_bot')
        self.publisher_ = self.create_publisher(String, 'status_updates', 10)
        timer_period = 2.0  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = 'A local federal MP is in hot water after being accused of threatening to punch a reporter at a pro-Australia rally'
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.data}"')
        self.i += 1

def main(args=None):
    rclpy.init(args=args)
    pineapple_gossip_bot = PineappleGossipBot()
    rclpy.spin(pineapple_gossip_bot)
    pineapple_gossip_bot.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
