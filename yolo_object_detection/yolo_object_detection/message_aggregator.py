import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool, Float32

class MessageAggregator(Node):
    def __init__(self):
        super().__init__('message_aggregator')

        self.last_values = {
            '/status': None,
            '/distance_ultra': None,
            'system_shutdown': None,
            'shutdown_feedback': set()
        }

        self.publisher = self.create_publisher(String, '/aggregated_status', 10)

        self.create_subscription(String, '/status', self.on_string('/status'), 10)
        self.create_subscription(Float32, '/distance_ultra', self.on_float('/distance_ultra'), 10)
        self.create_subscription(String, 'system_shutdown', self.on_string('system_shutdown'), 10)
        self.create_subscription(String, 'shutdown_feedback', self.on_feedback, 10)

    def on_string(self, topic):
        def callback(msg):
            if self.last_values[topic] != msg.data:
                self.last_values[topic] = msg.data
                self.publish(f"{msg.data}")
        return callback

    def on_float(self, topic):
        def callback(msg):
            old = self.last_values[topic]
            new = round(msg.data, 2)
            if old is None or abs(old - new) > 0.05:
                self.last_values[topic] = new
                self.publish(f"Abstand geändert: {new} m")
        return callback


    def on_feedback(self, msg):
        node = msg.data.strip()
        if node not in self.last_values['shutdown_feedback']:
            self.last_values['shutdown_feedback'].add(node)
            self.publish(f"✅ Shutdown-Bestätigung von: {node}")

    def publish(self, text):
        out = String()
        out.data = text
        self.publisher.publish(out)
        self.get_logger().info(text)

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
