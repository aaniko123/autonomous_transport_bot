import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
import cv2
import numpy as np

class ColorTracker(Node):
    def __init__(self):
        super().__init__('color_tracker')
        self.bridge = CvBridge()
        self.sub = self.create_subscription(Image, '/camera/image_raw', self.image_callback, 10)
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.mask_pub = self.create_publisher(Image, '/color_mask', 10)
        self.get_logger().info("Color Tracker Node Started")

    def image_callback(self, msg):
        img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

        # HSV range for a test color (greenish). Adjust if needed.
        lower = np.array([0, 50, 50])
        upper = np.array([179, 255, 255])

        mask = cv2.inRange(hsv, lower, upper)
        self.mask_pub.publish(self.bridge.cv2_to_imgmsg(mask, encoding='mono8'))

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        twist = Twist()
        if contours:
            c = max(contours, key=cv2.contourArea)
            M = cv2.moments(c)
            if M['m00'] > 0:
                cx = int(M['m10'] / M['m00'])
                self.get_logger().info(f"Object X: {cx}")
                img_center = img.shape[1] // 2
                error = cx - img_center

                twist.linear.x = 0.2
                twist.angular.z = -0.002 * error
        else:
            twist.linear.x = 0.0
            twist.angular.z = 0.0

        self.pub.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = ColorTracker()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
