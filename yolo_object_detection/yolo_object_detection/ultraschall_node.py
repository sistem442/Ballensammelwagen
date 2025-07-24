import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_msgs.msg import Float32, String
import RPi.GPIO as GPIO
import time
import threading

TRIG = 23
ECHO = 24

class UltraschallNode(Node):
    def __init__(self):
        super().__init__('ultraschall_node')
        self.running = True
        self.shutting_down = False

        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        self.publisher_ = self.create_publisher(Float32, '/distance_ultra', qos)
        self.feedback_pub = self.create_publisher(String, 'shutdown_feedback', 10)
        self.shutdown_sub = self.create_subscription(String, 'system_shutdown', self.shutdown_callback, 10)

        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)
        GPIO.setup(TRIG, GPIO.OUT)
        GPIO.setup(ECHO, GPIO.IN)
        GPIO.output(TRIG, False)

        self.get_logger().info("📡 Starte HC-SR04 Mess-Thread…")
        self.thread = threading.Thread(target=self.distance_loop)
        self.thread.start()

    def shutdown_callback(self, msg):
        if msg.data == "shutdown" and not self.shutting_down:
            self.shutting_down = True
            self.get_logger().info("🛑 Shutdown-Signal empfangen – stoppe Sensor")

            self.running = False  # Thread beenden
            self.thread.join()
            GPIO.cleanup()
            self.get_logger().info("🧹 HC-SR04 gestoppt und GPIO aufgeräumt.")

            confirm = String()
            confirm.data = 'ultraschall'
            self.feedback_pub.publish(confirm)

    def distance_loop(self):
        while self.running and rclpy.ok():
            GPIO.output(TRIG, False)
            time.sleep(0.1)
            GPIO.output(TRIG, True)
            time.sleep(0.00003)
            GPIO.output(TRIG, False)

            start_wait = time.time()
            timeout = start_wait + 0.1
            while GPIO.input(ECHO) == 0 and time.time() < timeout:
                startTime = time.time()
            if time.time() >= timeout:
                continue

            timeout = time.time() + 0.1
            while GPIO.input(ECHO) == 1 and time.time() < timeout:
                arrivalTime = time.time()
            if time.time() >= timeout:
                continue

            timeElapsed = arrivalTime - startTime
            distance_cm = (timeElapsed * 34300) / 2
            distance_m = round(distance_cm / 100.0, 3)

            if 0.02 < distance_m < 4.0 and not self.shutting_down:
                msg = Float32()
                msg.data = distance_m
                self.publisher_.publish(msg)

    def destroy_node(self):
        self.running = False
        if self.thread.is_alive():
            self.thread.join()
        GPIO.cleanup()
        self.get_logger().info("🧹 Node beendet & GPIO bereinigt")
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = UltraschallNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
