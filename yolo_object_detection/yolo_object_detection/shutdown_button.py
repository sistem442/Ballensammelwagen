import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import RPi.GPIO as GPIO

BUTTON_PIN = 17
EXPECTED_CONFIRMATIONS = ['yolo_detector', 'motion_control', 'bale_approach', 'ultraschall']  # Beispiel-Namen

class ShutdownButtonNode(Node):
    def __init__(self):
        super().__init__('system_shutdown')

        self.shutdown_pub = self.create_publisher(String, 'system_shutdown', 10)
        self.confirm_sub = self.create_subscription(String, 'shutdown_feedback', self.confirmation_callback, 10)

        GPIO.setmode(GPIO.BCM)
        GPIO.setup(BUTTON_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        self.timer = self.create_timer(0.2, self.check_button)

        self.shutdown_sent = False
        self.received_confirmations = set()

    def check_button(self):
        if GPIO.input(BUTTON_PIN) == GPIO.LOW and not self.shutdown_sent:
            self.get_logger().warn("🛎️ Knopf gedrückt – sende Shutdown-Signal")
            msg = String()
            msg.data = 'shutdown'
            self.shutdown_pub.publish(msg)
            self.shutdown_sent = True
            self.get_logger().info("📡 Shutdown-Nachricht gesendet, warte auf Bestätigungen ...")

    def confirmation_callback(self, msg):
        node_name = msg.data.strip()
        if node_name not in self.received_confirmations:
            self.received_confirmations.add(node_name)
            self.get_logger().info(f"✅ Bestätigung empfangen von: {node_name}")

        if all(name in self.received_confirmations for name in EXPECTED_CONFIRMATIONS):
            self.get_logger().info("🎯 Alle Nodes haben bestätigt – Shutdown wird durchgeführt")
            rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    node = ShutdownButtonNode()
    try:
        rclpy.spin(node)
    finally:
        GPIO.cleanup()
        node.destroy_node()
