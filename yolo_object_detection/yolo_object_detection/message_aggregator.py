import rclpy
from rclpy.node import Node
from messages.msg import CustomStatus

class MessageAggregator(Node):
    def __init__(self):
        super().__init__('message_aggregator')

        self.last_messages = {}

        self.publisher = self.create_publisher(CustomStatus, '/aggregated_status', 10)

        self.create_subscription(CustomStatus, '/status', self.on_status, 10)

    def on_status(self, msg):
        key = msg.node_name
        content = msg.data

        if self.last_messages.get(key) != content:
            self.last_messages[key] = content
            self.publish(key, content)

    def publish(self, sender, content):
        msg = CustomStatus()
        msg.node_name = sender
        msg.data = content
        self.publisher.publish(msg)
        self.get_logger().info(f"[{sender}] → {content}")

def main(args=None):
    rclpy.init(args=args)
    node = MessageAggregator()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
