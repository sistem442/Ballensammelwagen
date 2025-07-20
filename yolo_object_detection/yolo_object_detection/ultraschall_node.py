import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32

import RPi.GPIO as GPIO
import time
import threading
from rclpy.qos import QoSProfile, ReliabilityPolicy

TRIG = 23
ECHO = 24

class UltraschallNode(Node):
    def __init__(self):
        super().__init__('ultraschall_node')
        ultra_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        self.publisher_ = self.create_publisher(Float32, '/distance_ultra', ultra_qos)

        GPIO.setmode(GPIO.BCM)
        GPIO.setup(TRIG, GPIO.OUT)
        GPIO.setup(ECHO, GPIO.IN)
        GPIO.output(TRIG, False)

        self.running = True
        self.get_logger().info('Starte Mess-Thread für HC-SR04...')
        self.thread = threading.Thread(target=self.distance)
        self.thread.start()
       
    def distance(self):
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
                self.get_logger().warn("ECHO blieb LOW – kein Start erkannt")
                continue

            timeout = time.time() + 0.1
            while GPIO.input(ECHO) == 1 and time.time() < timeout:
                arrivalTime = time.time()
            if time.time() >= timeout:
                self.get_logger().warn("ECHO blieb HIGH – kein Ende erkannt")
                continue

            timeElapsed = arrivalTime - startTime
            distance_cm = (timeElapsed * 34300) / 2
            distance_m = round(distance_cm / 100.0, 3)

            #self.get_logger().info(f'Distanz: {distance_m:.2f} m')

            if 0.02 < distance_m < 4.0:
                if rclpy.ok():
                    msg = Float32()
                    msg.data = distance_m
                    self.publisher_.publish(msg)
            else:
                self.get_logger().warn(f'Ungültige Distanz: {distance_m:.2f} m')
    
    def destroy_node(self):
        self.running = False
        self.thread.join()
        GPIO.cleanup()
        self.get_logger().info('HC-SR04 gestoppt und GPIO aufgeräumt.')
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = UltraschallNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()

