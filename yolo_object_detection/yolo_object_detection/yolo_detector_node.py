import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, String
from cv_bridge import CvBridge
import cv2
from ultralytics import YOLO

class YoloDetector(Node):
    def __init__(self):
        super().__init__('yolo_detector')
        self.shutting_down = False
        self.bridge = CvBridge()
        self.model = YOLO('/home/boris/turtlebot3_ws/src/yolo_model/best.pt')

        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)

        self.image_sub = self.create_subscription(Image, '/image_raw', self.image_callback, qos)
        self.image_pub = self.create_publisher(Image, '/annotated_image', 10)
        self.approach_pub = self.create_publisher(Bool, '/approach_status', 10)
        self.status_pub = self.create_publisher(String, '/status', 10)
        self.feedback_pub = self.create_publisher(String, 'shutdown_feedback', 10)
        self.shutdown_sub = self.create_subscription(String, 'system_shutdown', self.shutdown_callback, 10)

        self.get_logger().info("🧠 YOLO Detector bereit")

    def shutdown_callback(self, msg):
        if msg.data == "shutdown" and not self.shutting_down:
            self.shutting_down = True
            self.get_logger().info("🛑 Shutdown-Signal empfangen – räume auf")

            # Rückmeldung senden
            confirm = String()
            confirm.data = 'yolo_detector'
            self.feedback_pub.publish(confirm)

    def image_callback(self, msg):
        if self.shutting_down or not rclpy.ok():
            return

        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            results = self.model(cv_image, verbose=False)[0]

            msg = String()
            msg.data = ''

            approaching = False
            for det in results.boxes:
                cls = int(det.cls)
                label = self.model.names[cls]
                conf = float(det.conf)

                if conf > 0.5 and label.lower() == 'bale':
                    x1, y1, x2, y2 = map(int, det.xyxy[0])
                    box_height = y2 - y1
                    approaching = box_height < 300

                    cv2.rectangle(cv_image, (x1, y1), (x2, y2), (0, 255, 0), 2)
                    cv2.putText(cv_image, f"{label} ({conf:.2f})", (x1, y1 - 10),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)

                    # self.get_logger().info(
                    #     f"🎯 Ballen erkannt – Höhe: {box_height}px – approaching={approaching}",
                    #     throttle_duration_sec=5.0
                    # )
                    break
            #zum message_aggregator melden
            if approaching:
                msg.data = 'Yolo Node: Ballen erkannt'
                self.status_pub.publish(msg)
            else:
                msg.data = 'Yolo Node: Ballen nicht erkannt '
                self.status_pub.publish(msg)

            #zum motion_control melden
            status_msg = Bool()
            status_msg.data = approaching
            self.approach_pub.publish(status_msg)

            img_out = self.bridge.cv2_to_imgmsg(cv_image, encoding='bgr8')
            self.image_pub.publish(img_out)

        except Exception as e:
            self.get_logger().error(f"⚠️ YOLO-Fehler: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = YoloDetector()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
