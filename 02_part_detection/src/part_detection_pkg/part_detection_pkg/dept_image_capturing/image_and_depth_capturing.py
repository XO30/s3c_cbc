import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
import os

class DepthMapNode(Node):
    def __init__(self):
        super().__init__('depth_map_node')

        self.bridge = CvBridge()
        self.key_pressed = False
        self.depth_map_count = 1

        self.subdirectory = 'images_depth'
        if not os.path.exists(self.subdirectory):
            os.makedirs(self.subdirectory)

        self.depth_map_sub = self.create_subscription(
            Image, 
            '/camera/depth/image_rect_raw', 
            self.depth_map_callback, 
            10
        )

        self.rgb_image = None
        self.rgb_sub = self.create_subscription(
            Image, 
            '/camera/color/image_raw', 
            self.rgb_callback, 
            10
        )

        self.create_timer(0.1, self.check_key_press)

    def depth_map_callback(self, msg):
        try:
            depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='32FC1')
            depth_image[np.isnan(depth_image)] = 255.0

            min_depth = 0
            max_depth = 800
            scale_factor = 410 / (max_depth - min_depth)
            depth_image_scaled = (depth_image - min_depth) * scale_factor
            depth_image_scaled = np.clip(depth_image_scaled, 0, 255)
            depth_image_scaled = depth_image_scaled.astype(np.uint8)

            cv2.imshow("Depth Map Viewer", depth_image_scaled)

            key = cv2.waitKey(10)
            if key == ord('c'):
                self.save_images(depth_image, depth_image_scaled)
            elif key == 27:
                self.key_pressed = True

        except Exception as e:
            self.get_logger().error(f"Error: {e}")

    def rgb_callback(self, msg):
        self.rgb_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

    def save_images(self, depth_image, depth_image_scaled):
        filename_image = os.path.join(self.subdirectory, f"new_{self.depth_map_count}.png")
        cv2.imwrite(filename_image, depth_image_scaled)
        self.get_logger().info(f"Depth map saved as {filename_image}")

        filename_array = os.path.join(self.subdirectory, f"new_{self.depth_map_count}.npy")
        np.save(filename_array, depth_image)
        self.get_logger().info(f"Depth data saved as {filename_array}")

        filename_array_scaled = os.path.join(self.subdirectory, f"new_scaled_{self.depth_map_count}.npy")
        np.save(filename_array_scaled, depth_image_scaled)
        self.get_logger().info(f"Scaled depth data saved as {filename_array_scaled}")

        if self.rgb_image is not None:
            rgb_filename = os.path.join(self.subdirectory, f"new_rgb_{self.depth_map_count}.png")
            cv2.imwrite(rgb_filename, self.rgb_image)
            self.get_logger().info(f"RGB image saved as {rgb_filename}")

        self.depth_map_count += 1

    def check_key_press(self):
        if self.key_pressed:
            rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    node = DepthMapNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
