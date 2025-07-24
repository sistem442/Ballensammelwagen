import rclpy
from rclpy.node import Node
from rclpy.logging import get_logger
from rclpy.qos import QoSProfile, ReliabilityPolicy

from std_msgs.msg import Float32, Bool, String
from geometry_msgs.msg import Twist

class MotionControl(Node):
    def __init__(self):
        super().__init__('motion_control')

        self.logger = get_logger('motion_control')

        ultra_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        self.ultra_sub = self.create_subscription(Float32, '/distance_ultra', self.ultrasonic_callback, ultra_qos)

        yolo_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        self.approach_sub = self.create_subscription(Bool, '/approach_status', self.yolo_callback, yolo_qos)

        cmd_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', cmd_qos)

        status_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        self.ballen_status = self.create_publisher(String, '/ballen_status', status_qos)

        self.approaching = False
        self.ultra_too_close = False
        self.last_twist = Twist()

        self.create_timer(0.1, self.send_last_twist)
        self.shutdown_sub = self.create_subscription(String, 'system_shutdown', self.shutdown_callback, 10)

    def shutdown_callback(self, msg):
        if msg.data == "shutdown":
            self.logger.info("🛑 Shutdown-Signal empfangen – Node wird sauber beendet")
            rclpy.shutdown()

    def ultrasonic_callback(self, msg):
        distance = msg.data
        self.ultra_too_close = distance < 0.3
        if self.ultra_too_close:
            self.logger.info(
                f"🛑 Ultraschall: {distance:.2f}m – Stop!",
                throttle_duration_sec=5.0
            )
            self.approaching = False
        else:
            self.logger.info(
                f"📏 Ultraschall: {distance:.2f}m – Weiterfahren möglich",
                throttle_duration_sec=5.0
            )

    def yolo_callback(self, msg):
        self.approaching = msg.data
        status = "aktiviert" if msg.data else "deaktiviert"
        self.logger.info(
            f"🧠 YOLO Approaching: {status}",
            throttle_duration_sec=5.0
        )

    def send_last_twist(self):
        if self.approaching and not self.ultra_too_close:
            self.last_twist.linear.x = 0.05
            self.cmd_pub.publish(self.last_twist)
            self.logger.info("🚀 Fahre vorwärts…", throttle_duration_sec=5.0)
        elif self.approaching and self.ultra_too_close:
            msg = String()
            msg.data = "ballen_erkannt"
            self.ballen_status.publish(msg)
            self.logger.info(
                "📢 Ballen erkannt und in Nähe – Nachricht veröffentlicht",
                throttle_duration_sec=5.0
            )
        else:
            self.cmd_pub.publish(Twist())
            self.logger.info(
                "🛑 Kein Befehl – Sicherheit oder kein Ballen",
                throttle_duration_sec=5.0
            )

def main(args=None):
    rclpy.init(args=args)
    node = MotionControl()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.logger.info("Abbruch erkannt – Node wird beendet.")
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()

if __name__ == '__main__':
    main()
