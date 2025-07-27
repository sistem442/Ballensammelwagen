import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_msgs.msg import Float32, Bool, String
from geometry_msgs.msg import Twist
from messages.msg import CustomStatus

class MotionControl(Node):
    def __init__(self):
        super().__init__('motion_control')
        self.get_logger().info("🚦 Motion Control Node gestartet")

        # Status-Flags
        self.approaching = False
        self.ultra_too_close = False
        self.shutting_down = False

        # Publisher
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', qos)
        self.approach_pub = self.create_publisher(Bool, '/ballen_approach', qos)
        self.feedback_pub = self.create_publisher(String, 'shutdown_feedback', 10)
        self.status_pub = self.create_publisher(CustomStatus, 'status', 10)

        # Subscriber
        self.create_subscription(Float32, '/distance_ultra', self.ultrasonic_callback, qos)
        self.create_subscription(Bool, '/approach_status', self.yolo_callback, qos)
        self.create_subscription(String, 'system_shutdown', self.shutdown_callback, 10)

        # Steuer-Timer
        self.control_timer = self.create_timer(0.1, self.send_last_twist)

    def shutdown_callback(self, msg):
        if msg.data == "shutdown" and not self.shutting_down:
            self.shutting_down = True
            self.get_logger().info("🛑 Shutdown-Signal empfangen – räume auf")

            # Timer stoppen
            self.control_timer.cancel()

            # Roboter sicher stoppen
            self.cmd_pub.publish(Twist())

            # Feedback senden
            confirm = String()
            confirm.data = 'motion_control'
            self.get_logger().info("📤 Shutdown-Bestätigung wurde veröffentlicht")
            self.feedback_pub.publish(confirm)

    def ultrasonic_callback(self, msg):
        if self.shutting_down:
            return
        distance = msg.data
        self.ultra_too_close = distance < 0.2
        if self.ultra_too_close:
            self.approaching = False

    def yolo_callback(self, msg):
        if self.shutting_down:
            return
        self.approaching = msg.data

    def send_last_twist(self):
        if self.shutting_down:
            return

        twist = Twist()
        msg = CustomStatus()

        if self.approaching and not self.ultra_too_close:
            twist.linear.x = 0.05
            self.cmd_pub.publish(twist)
            msg.data = 'Motion Control Node: Fahren zu dem Ballen'
            self.status_pub.publish(msg)
        elif self.approaching and self.ultra_too_close:
            status_msg = Bool()
            status_msg.data = True
            self.approach_pub.publish(status_msg)
            msg = CustomStatus()
            msg.node_name = "motion_control"
            msg.data = "Fahren links von dem Ballen"
            self.status_pub.publish(msg)
        else:
            self.cmd_pub.publish(twist)  # Stop-Befehl
            msg = CustomStatus()
            msg.node_name = "motion_control"
            msg.data = 'Stoppen'
            self.status_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = MotionControl()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
