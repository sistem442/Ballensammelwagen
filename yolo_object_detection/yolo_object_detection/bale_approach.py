import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist

class BaleApproach(Node):
    def __init__(self):
        super().__init__('bale_approach')

        self.step_index = 0
        self.movement_sequence = [
            ('forward', 2.0),
            ('turn_right', 0.8),
            ('forward', 1.5),
            ('turn_left', 0.8),
            ('forward', 1.5),
            ('stop', 0.2)
        ]

        self.shutting_down = False
        self.active_timer = None

        # Kommunikation
        self.subscription = self.create_subscription(String, 'ballen_status', self.listener_callback, 10)
        self.shutdown_sub = self.create_subscription(String, 'system_shutdown', self.shutdown_callback, 10)
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.feedback_pub = self.create_publisher(String, 'shutdown_feedback', 10)

    def shutdown_callback(self, msg):
        if msg.data == "shutdown" and not self.shutting_down:
            self.shutting_down = True
            self.get_logger().info("🛑 Shutdown-Signal empfangen – unterbreche Bewegungssequenz")

            # Bewegung stoppen
            self.cmd_pub.publish(Twist())

            # Timer abbrechen, falls aktiv
            if self.active_timer:
                self.active_timer.cancel()

            # Bestätigung senden
            confirm = String()
            confirm.data = 'bale_approach'
            self.feedback_pub.publish(confirm)

    def listener_callback(self, msg):
        if msg.data == "ballen_erkannt" and not self.shutting_down:
            self.get_logger().info("📨 Ballen erkannt – beginne Approach-Sequenz")
            self.step_index = 0
            self.execute_next_step()

    def execute_next_step(self):
        if self.shutting_down:
            return

        if self.step_index >= len(self.movement_sequence):
            self.get_logger().info("✅ Maneuver abgeschlossen")
            self.cmd_pub.publish(Twist())
            return

        action, duration = self.movement_sequence[self.step_index]
        twist = Twist()

        if action == 'forward':
            twist.linear.x = 0.15
        elif action == 'turn_right':
            twist.angular.z = -0.3
        elif action == 'turn_left':
            twist.angular.z = 0.3
        elif action == 'stop':
            twist = Twist()

        self.cmd_pub.publish(twist)
        self.get_logger().info(f"🚦 Schritt {self.step_index + 1}: {action} für {duration} s")

        self.step_index += 1
        self.active_timer = self.create_timer(duration, self.execute_next_step)

def main(args=None):
    rclpy.init(args=args)
    node = BaleApproach()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
