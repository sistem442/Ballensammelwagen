# motion_control_node.py
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.qos import QoSProfile, ReliabilityPolicy

from std_msgs.msg import Float32, Bool
from geometry_msgs.msg import Twist


class MotionControl(Node):
    def __init__(self):
        super().__init__('motion_control')

        ultra_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        self.ultra_sub = self.create_subscription(Float32, '/distance_ultra', self.ultrasonic_callback, ultra_qos)

        yolo_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        self.approach_sub = self.create_subscription(Bool, '/approach_status', self.yolo_callback, yolo_qos)

        cmd_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', cmd_qos)

        self.approaching = False
        self.ultra_too_close = False
        self.last_twist = Twist()

        self.create_timer(0.1, self.send_last_twist)

    def ultrasonic_callback(self, msg):
        distance = msg.data
        self.ultra_too_close = distance < 0.15
        if self.ultra_too_close:
            self.get_logger().info(f"🛑 Ultraschall: {distance:.2f}m – Stop!")
            self.approaching = False
        else:
            self.get_logger().info(f"📏 Ultraschall: {distance:.2f}m – Weiterfahren möglich")

    def yolo_callback(self, msg):
        self.approaching = msg.data
        status = "aktiviert" if msg.data else "deaktiviert"
        self.get_logger().info(f"🧠 YOLO Approaching: {status}")

    def send_last_twist(self):
        if self.approaching and not self.ultra_too_close:
            self.last_twist.linear.x = 0.05
            self.cmd_pub.publish(self.last_twist)
            self.get_logger().info("🚀 Fahre vorwärts…")
        else:
            self.cmd_pub.publish(Twist())
            self.get_logger().info("🛑 Kein Befehl – Sicherheit oder kein Ballen")


def main(args=None):
    rclpy.init(args=args)
    node = MotionControl()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Abbruch erkannt – Node wird beendet.")
    finally:
        if rclpy.ok():  # Nur wenn ROS-Kontext noch aktiv ist
            node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()
