import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, String
from messages.msg import CustomStatus
import math


class BaleApproach(Node):
    def __init__(self):
        super().__init__('bale_approach_odom')

        self.step_index = 0
        self.shutting_down = False
        self.start_position = None
        self.current_position = None

        self.movement_sequence = [
            ('turn_right', math.radians(45)),
            ('forward', 0.2),
            ('turn_left', math.radians(45)),
            ('forward', 0.20),
            ('stop', 0.0)
        ]

        # Subscriptions
        self.shutdown_sub = self.create_subscription(String, 'system_shutdown', self.shutdown_callback, 10)
        self.trigger_sub = self.create_subscription(Bool, 'ballen_approach', self.listener_callback, 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)

        # Publishers
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.feedback_pub = self.create_publisher(String, 'shutdown_feedback', 10)
        self.status_pub = self.create_publisher(CustomStatus, 'status', 10)

    def shutdown_callback(self, msg):
        if msg.data == "shutdown":
            self.shutting_down = True
            self.cmd_pub.publish(Twist())
            confirm = String()
            confirm.data = 'bale_approach'
            self.feedback_pub.publish(confirm)
            self.get_logger().info("🛑 Shutdown empfangen. Bewegungsabbruch.")

    def listener_callback(self, msg):
        if msg.data and not self.shutting_down:
            self.step_index = 0
            self.start_position = None
            self.get_logger().info("🎯 Trigger empfangen. Starte Anfahrt.")

    def odom_callback(self, msg):
        if self.shutting_down or self.step_index >= len(self.movement_sequence):
            return

        position = msg.pose.pose.position
        orientation = msg.pose.pose.orientation

        if self.start_position is None:
            self.start_position = position
            self.start_yaw = self.quaternion_to_yaw(orientation)
            self.execute_movement()

        else:
            dx = position.x - self.start_position.x
            dy = position.y - self.start_position.y
            dist = math.sqrt(dx ** 2 + dy ** 2)
            yaw = self.quaternion_to_yaw(orientation)
            dyaw = abs(self.normalize_angle(yaw - self.start_yaw))

            action, target = self.movement_sequence[self.step_index]

            if action == 'forward' and dist >= target:
                self.advance_step()
            elif action == 'turn_left' and dyaw >= target:
                self.advance_step()
            elif action == 'turn_right' and dyaw >= target:
                self.advance_step()

    def execute_movement(self):
        twist = Twist()
        action, _ = self.movement_sequence[self.step_index]

        if action == 'forward':
            twist.linear.x = 0.05
        elif action == 'turn_left':
            twist.angular.z = 0.3
        elif action == 'turn_right':
            twist.angular.z = -0.3
        elif action == 'stop':
            twist = Twist()
            self.status_pub.publish(self.build_status("Anfahrt abgeschlossen"))
            self.step_index = len(self.movement_sequence)
            return

        self.cmd_pub.publish(twist)

    def advance_step(self):
        self.cmd_pub.publish(Twist())
        self.step_index += 1
        self.start_position = None
        if self.step_index < len(self.movement_sequence):
            self.get_logger().info(f"🔄 Schritt {self.step_index} starten…")
        else:
            self.get_logger().info("✅ Alle Schritte abgeschlossen.")

    def build_status(self, text):
        status = CustomStatus()
        status.node_name = 'bale_approach'
        status.data = text
        return status

    def quaternion_to_yaw(self, q):
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny, cosy)

    def normalize_angle(self, angle):
        return math.atan2(math.sin(angle), math.cos(angle))


def main(args=None):
    rclpy.init(args=args)
    node = BaleApproach()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
