#!/usr/bin/env python3

import cv2
from cv_bridge import CvBridge
import rclpy
from rclpy.utilities import ok as rclpy_ok
from rclpy.node import Node
from sensor_msgs.msg import Image
import os
import time


class ImageUtilNode(Node):
    def __init__(self):
        super().__init__('image_util_node')
        
        # topic parameter
        self.declare_parameter('rgb_image_topic', '/camera/color/image_raw')
        self.rgb_image_topic = self.get_parameter('rgb_image_topic').get_parameter_value().string_value

        self.bridge = CvBridge()
        self.subscription = self.create_subscription(
            Image,
            self.rgb_image_topic,
            self.image_callback,
            10)
        
        self.get_logger().info(f'Subscribed to {self.rgb_image_topic}')

        self.image = None

        # Create OpenCV window and register mouse callback
        cv2.namedWindow("Image", cv2.WINDOW_NORMAL)
        cv2.setMouseCallback("Image", self.mouse_callback)

        # Directory to save images
        self.save_dir = "saved_images"
        os.makedirs(self.save_dir, exist_ok=True)

    def image_callback(self, msg):
        """ROS image callback"""
        self.image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

    def view_image(self):
        """Continuously display frames"""
        if self.image is not None:
            cv2.imshow("Image", self.image)
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            return False  # exit loop
        return True

    def mouse_callback(self, event, x, y, flags, param):
        """Save image on Ctrl + Right Click"""
        if event == cv2.EVENT_RBUTTONDOWN and (flags & cv2.EVENT_FLAG_CTRLKEY):
            if self.image is not None:
                filename = os.path.join(self.save_dir, f"frame_{int(time.time())}.png")
                cv2.imwrite(filename, self.image)
                self.get_logger().info(f"Saved image (Ctrl+Right Click): {filename}")


def main(args=None):
    rclpy.init(args=args)
    node = ImageUtilNode()

    try:
        while rclpy_ok():
            rclpy.spin_once(node, timeout_sec=0.01)
            if not node.view_image():
                break
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()

