import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import RPi.GPIO as GPIO

BUTTON_PIN = 17  # oder dein GPIO

class ShutdownButtonNode(Node):
    def __init__(self):
        super().__init__('system_shutdown')  # Node-Name

        # ROS2 Publisher auf /system_shutdown
        self.publisher_ = self.create_publisher(String, 'system_shutdown', 10)

        # GPIO vorbereiten
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(BUTTON_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)

        # Timer zum Abfragen des Knopfs
        self.timer = self.create_timer(0.2, self.check_button)

        # Statusvariable
        self.shutdown_sent = False

    def check_button(self):
        if GPIO.input(BUTTON_PIN) == GPIO.LOW and not self.shutdown_sent:
            self.get_logger().warn("🛎️ Knopf gedrückt – sende Shutdown-Signal")
            msg = String()
            msg.data = "shutdown"
            self.publisher_.publish(msg)

            self.shutdown_sent = True
            self.get_logger().info("⏳ Shutdown läuft ...")

            # Optional: Selbst beenden
            rclpy.shutdown()

    def on_shutdown(self):
        print("✅ Shutdown abgeschlossen – Node wird beendet")

def main(args=None):
    rclpy.init(args=args)
    node = ShutdownButtonNode()

    # Callback bei Shutdown registrieren
    rclpy.get_default_context().on_shutdown(node.on_shutdown)

    try:
        rclpy.spin(node)
    finally:
        GPIO.cleanup()
        node.destroy_node()
