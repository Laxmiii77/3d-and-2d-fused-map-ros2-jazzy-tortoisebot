import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32
import math

class LaserScanProcessor(Node):
    def __init__(self):
        super().__init__('laser_scan_processor')

    
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',  
            self.laser_scan_callback,
            10)

        self.distance_pub = self.create_publisher(Float32, '/front_distance', 10)

        self.current_velocity = Twist()

    def laser_scan_callback(self, msg):
        front_index = len(msg.ranges) // 2
        front_distance = msg.ranges[front_index]

        distance_msg = Float32()
        distance_msg.data = front_distance
        self.distance_pub.publish(distance_msg)
        if math.isinf(front_distance):
            pass
        else:
            self.get_logger().info(f'Object distance: {front_distance:.2f} m')

def main(args=None):
    rclpy.init(args=args)
    node = LaserScanProcessor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
