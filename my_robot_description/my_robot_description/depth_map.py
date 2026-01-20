import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, Image
import sensor_msgs_py.point_cloud2 as pc2
from cv_bridge import CvBridge
import numpy as np
import cv2

class PointCloudToDepthMap(Node):
    def __init__(self):
        super().__init__('point_cloud_to_depth_map')
        self.subscription = self.create_subscription(PointCloud2,'/depth_camera/points',self.point_cloud_callback,10)
        self.publisher_ = self.create_publisher(Image, '/depth_map', 10)
        self.bridge = CvBridge()
        self.get_logger().info('PointCloud to Depth Map Node has been started.')
        # Parameters for depth map
        self.width = 750 # Width of the depth image
        self.height = 500 # Height of the depth image
        self.scale = 100 # Scale factor to convert meters to pixels

    def point_cloud_callback(self, msg):
        # Read points from the point cloud message
        points = pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)
        # Create an empty depth map
        depth_map = np.zeros((self.height, self.width), dtype=np.uint8)
        # Define the center of the depth map
        center_x = self.width // 2
        center_y = self.height // 2

        for point in points:
            x, y, z = point
            # Filter out infinite and NaN values
            if not np.isfinite(x) or not np.isfinite(y) or not np.isfinite(z):
                continue # Skip the point if it contains NaN or infinite values
            
            # CORRECTED: Swap x and y coordinates to fix rotation
            pixel_x = int(center_x - y * self.scale)  # Use -y for horizontal position
            pixel_y = int(center_y - x * self.scale)  # Use -x for vertical position
            
            # Check if the pixel is within image bounds
            if 0 <= pixel_x < self.width and 0 <= pixel_y < self.height:
                # Normalize depth value to 0-255
                depth_value = np.clip(z * 255 / 50, 0, 255) # Assuming max depth of 50 meters
                depth_map[pixel_y, pixel_x] = 255-int(depth_value) # Inverting so closer points are brighter
        
        # Convert the depth map to a ROS Image message
        image_msg = self.bridge.cv2_to_imgmsg(depth_map, encoding="mono8")
        # Publish the depth map
        self.publisher_.publish(image_msg)
        # self.get_logger().info('Published depth map.')

def main(args=None):
    rclpy.init(args=args)
    node = PointCloudToDepthMap()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()