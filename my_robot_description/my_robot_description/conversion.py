#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from sensor_msgs.msg import LaserScan, PointCloud2
from laser_geometry import LaserProjection
import tf2_ros
from tf2_ros import TransformException
from rclpy.time import Time


class LaserScanToPointCloudNode(Node):
    def __init__(self):
        super().__init__('laserscan_to_pointcloud')

        # Parameters
        self.target_frame = self.declare_parameter('target_frame', '').value  # leave empty to keep laser frame
        self.queue_size = self.declare_parameter('queue_size', 10).value

        # Laser projector
        self.projector = LaserProjection()

        # Publisher (use RELIABLE for RViz/Octomap)
        self.pub = self.create_publisher(
            PointCloud2,
            'cloud',
            QoSProfile(
                reliability=ReliabilityPolicy.RELIABLE,
                history=HistoryPolicy.KEEP_LAST,
                depth=self.queue_size
            )
        )

        # TF buffer & listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Subscriber
        self.sub = self.create_subscription(
            LaserScan,
            'scan',
            self.scan_callback,
            QoSProfile(
                reliability=ReliabilityPolicy.BEST_EFFORT,
                history=HistoryPolicy.KEEP_LAST,
                depth=self.queue_size
            )
        )

        self.get_logger().info("LaserScanToPointCloudNode started")

    def scan_callback(self, scan_msg: LaserScan):
        try:
            if self.target_frame:
                # Project + transform directly into target_frame
                cloud_msg = self.projector.transformLaserScanToPointCloud(
                    self.target_frame,
                    scan_msg,
                    self.tf_buffer
                )
            else:
                # Keep in the laser frame
                cloud_msg = self.projector.projectLaser(scan_msg)
            
            self.pub.publish(cloud_msg)

        except TransformException as ex:
            self.get_logger().warn(f"Transform error: {ex}")
        except Exception as ex:
            self.get_logger().error(f"Unexpected error: {ex}")
def main(args=None):
    rclpy.init(args=args)
    node = LaserScanToPointCloudNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()