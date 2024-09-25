import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
import os

class DepthMapNode(Node):
    def __init__(self, save_directory):
        super().__init__('depth_map_node')

        self.bridge = CvBridge()
        self.depth_map_count = 1

        self.depth_image_dir = os.path.join(save_directory, 'depth_images')
        self.depth_data_dir = os.path.join(save_directory, 'depth_data')
        self.depth_data_scaled_dir = os.path.join(save_directory, 'depth_data_scaled')
        self.rgb_image_dir = os.path.join(save_directory, 'rgb_images')

        os.makedirs(self.depth_image_dir, exist_ok=True)
        os.makedirs(self.depth_data_dir, exist_ok=True)
        os.makedirs(self.depth_data_scaled_dir, exist_ok=True)
        os.makedirs(self.rgb_image_dir, exist_ok=True)

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

            self.save_images(depth_image, depth_image_scaled)

        except Exception as e:
            self.get_logger().error(f"Error: {e}")

    def rgb_callback(self, msg):
        self.rgb_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        self.save_rgb_image(self.rgb_image)

    def save_images(self, depth_image, depth_image_scaled):
        filename_image = os.path.join(self.depth_image_dir, f"new_{self.depth_map_count}.png")
        cv2.imwrite(filename_image, depth_image_scaled)
        self.get_logger().info(f"Depth map saved as {filename_image}")

        filename_array = os.path.join(self.depth_data_dir, f"new_{self.depth_map_count}.npy")
        np.save(filename_array, depth_image)
        self.get_logger().info(f"Depth data saved as {filename_array}")

        filename_array_scaled = os.path.join(self.depth_data_scaled_dir, f"new_scaled_{self.depth_map_count}.npy")
        np.save(filename_array_scaled, depth_image_scaled)
        self.get_logger().info(f"Scaled depth data saved as {filename_array_scaled}")

        self.depth_map_count += 1

    def save_rgb_image(self, rgb_image):
        filename_rgb = os.path.join(self.rgb_image_dir, f"new_rgb_{self.depth_map_count}.png")
        cv2.imwrite(filename_rgb, rgb_image)
        self.get_logger().info(f"RGB image saved as {filename_rgb}")

def main(args=None):
    rclpy.init(args=args)
    save_directory = '/media/stefan/backups/extracted_images'  # Replace with your absolute path
    node = DepthMapNode(save_directory)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
