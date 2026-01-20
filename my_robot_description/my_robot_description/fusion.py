#!/usr/bin/env python3
"""
Efficient PointCloud fusion node for ROS2.

- Vectorized PointCloud2 <-> numpy conversion
- Single TF lookup + matrix transform for entire cloud
- Uses latest available TF to avoid delays
- Publishes fused cloud in target_frame
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from rclpy.parameter import Parameter
from rclpy.time import Time

import numpy as np
import struct
import threading

import tf2_ros
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header


# ---------------- Helpers ----------------
def quaternion_to_rotation_matrix(qx, qy, qz, qw):
    """Return 3x3 rotation matrix from quaternion (x,y,z,w)."""
    norm = np.sqrt(qx*qx + qy*qy + qz*qz + qw*qw)
    if norm == 0:
        return np.eye(3)
    qx, qy, qz, qw = qx/norm, qy/norm, qz/norm, qw/norm
    xx, yy, zz = qx*qx, qy*qy, qz*qz
    xy, xz, yz = qx*qy, qx*qz, qy*qz
    wx, wy, wz = qw*qx, qw*qy, qw*qz

    return np.array([
        [1 - 2*(yy + zz), 2*(xy - wz),     2*(xz + wy)],
        [2*(xy + wz),     1 - 2*(xx + zz), 2*(yz - wx)],
        [2*(xz - wy),     2*(yz + wx),     1 - 2*(xx + yy)]
    ], dtype=np.float64)


def pointcloud2_to_xyz_numpy(cloud_msg):
    """Convert PointCloud2 to Nx3 numpy array of float32."""
    if cloud_msg is None or not cloud_msg.data or cloud_msg.point_step == 0:
        return np.empty((0, 3), dtype=np.float32)

    fields = {f.name: f for f in cloud_msg.fields}
    if not all(k in fields for k in ('x', 'y', 'z')):
        return np.empty((0, 3), dtype=np.float32)

    step = cloud_msg.point_step
    arr = np.frombuffer(cloud_msg.data, dtype=np.uint8)
    if arr.size % step != 0:
        return np.empty((0, 3), dtype=np.float32)

    pts = arr.reshape((-1, step))
    try:
        bx = pts[:, fields['x'].offset:fields['x'].offset+4].copy().view(np.float32).ravel()
        by = pts[:, fields['y'].offset:fields['y'].offset+4].copy().view(np.float32).ravel()
        bz = pts[:, fields['z'].offset:fields['z'].offset+4].copy().view(np.float32).ravel()
        xyz = np.stack([bx, by, bz], axis=1)
    except Exception:
        # fallback: slower per-point unpack
        xyz = []
        for i in range(0, len(cloud_msg.data), step):
            try:
                x = struct.unpack_from('f', cloud_msg.data, i + fields['x'].offset)[0]
                y = struct.unpack_from('f', cloud_msg.data, i + fields['y'].offset)[0]
                z = struct.unpack_from('f', cloud_msg.data, i + fields['z'].offset)[0]
                if np.isfinite(x) and np.isfinite(y) and np.isfinite(z):
                    xyz.append([x, y, z])
            except struct.error:
                continue
        return np.asarray(xyz, dtype=np.float32)

    mask = np.isfinite(xyz).all(axis=1)
    return xyz[mask]


def xyz_to_pointcloud2_numpy(points, frame_id, stamp):
    """Convert Nx3 numpy array to PointCloud2."""
    if points is None or len(points) == 0:
        return PointCloud2()

    points = np.asarray(points, dtype=np.float32)
    data = points.tobytes()
    step = 12

    msg = PointCloud2()
    msg.header = Header(stamp=stamp, frame_id=frame_id)
    msg.height = 1
    msg.width = points.shape[0]
    msg.fields = [
        PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
    ]
    msg.is_bigendian = False
    msg.point_step = step
    msg.row_step = step * msg.width
    msg.is_dense = True
    msg.data = data
    return msg


# ---------------- Node ----------------
class PointCloudFusionNode(Node):
    def __init__(self):
        super().__init__('pointcloud_fusion_fast')

        # Declare custom params
        self.declare_parameter('target_frame', 'odom')
        self.declare_parameter('expected_lidar_frame', 'tortoisebot/lidar_link/lidar_sensor')
        self.declare_parameter('expected_camera_frame', 'camera_link')
        self.declare_parameter('publish_rate_hz', 10.0)

        self.target_frame = self.get_parameter('target_frame').value
        self.expected_lidar_frame = self.get_parameter('expected_lidar_frame').value
        self.expected_camera_frame = self.get_parameter('expected_camera_frame').value
        publish_rate = float(self.get_parameter('publish_rate_hz').value)

        # QoS
        sensor_qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=10)
        reliable_qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST, depth=10)

        # Subs & pubs
        self.map_sub = self.create_subscription(PointCloud2, '/cloud', self.map_callback, sensor_qos)
        self.depth_sub = self.create_subscription(PointCloud2, '/depth_camera/points', self.depth_callback, sensor_qos)
        self.fused_pub = self.create_publisher(PointCloud2, '/fused_pointcloud', reliable_qos)

        # TF
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # state
        self.lock = threading.Lock()
        self.latest_map = None
        self.latest_depth = None
        self.map_frame = None
        self.depth_frame = None

        # stats
        self.map_count = 0
        self.depth_count = 0
        self.fusion_count = 0

        # timers
        self.create_timer(1.0 / max(0.001, publish_rate), self.fuse_and_publish)
        self.create_timer(5.0, self.print_status)

        self.get_logger().info("PointCloudFusionNode (fast) initialized")

    # ---------- logging ----------
    def print_status(self):
        self.get_logger().info(f"Map msgs: {self.map_count}, Depth msgs: {self.depth_count}, Fusions: {self.fusion_count}")

    # ---------- callbacks ----------
    def map_callback(self, msg):
        with self.lock:
            self.latest_map = msg
            self.map_frame = msg.header.frame_id
            self.map_count += 1

    def depth_callback(self, msg):
        with self.lock:
            self.latest_depth = msg
            self.depth_frame = msg.header.frame_id
            self.depth_count += 1

    # ---------- transform ----------
    def lookup_transform_matrix(self, from_frame, to_frame):
        if not from_frame or from_frame == to_frame:
            return np.eye(4, dtype=np.float64)
        try:
            tf_stamped = self.tf_buffer.lookup_transform(to_frame, from_frame, Time())
        except Exception as e:
            self.get_logger().warn(f"TF lookup failed {from_frame}->{to_frame}: {e}")
            return None

        t = tf_stamped.transform.translation
        q = tf_stamped.transform.rotation
        R = quaternion_to_rotation_matrix(q.x, q.y, q.z, q.w)
        T = np.eye(4)
        T[:3, :3] = R
        T[:3, 3] = [t.x, t.y, t.z]
        return T

    # ---------- fusion ----------
    def fuse_and_publish(self):
        with self.lock:
            map_msg = self.latest_map
            depth_msg = self.latest_depth
            map_frame = self.map_frame
            depth_frame = self.depth_frame

        if map_msg is None and depth_msg is None:
            return

        map_pts = pointcloud2_to_xyz_numpy(map_msg) if map_msg else np.empty((0, 3), np.float32)
        depth_pts = pointcloud2_to_xyz_numpy(depth_msg) if depth_msg else np.empty((0, 3), np.float32)

        # Transform to target_frame
        if map_pts.size and map_frame != self.target_frame:
            T_map = self.lookup_transform_matrix(map_frame, self.target_frame)
            if T_map is not None:
                homog = np.hstack([map_pts, np.ones((map_pts.shape[0], 1))])
                map_pts = (T_map @ homog.T).T[:, :3].astype(np.float32)

        if depth_pts.size and depth_frame != self.target_frame:
            T_depth = self.lookup_transform_matrix(depth_frame, self.target_frame)
            if T_depth is not None:
                homog = np.hstack([depth_pts, np.ones((depth_pts.shape[0], 1))])
                depth_pts = (T_depth @ homog.T).T[:, :3].astype(np.float32)

        # Fuse
        if map_pts.size and depth_pts.size:
            fused = np.vstack([map_pts, depth_pts])
        elif map_pts.size:
            fused = map_pts
        elif depth_pts.size:
            fused = depth_pts
        else:
            return

        # Clean
        mask = np.isfinite(fused).all(axis=1)
        fused = fused[mask]
        if fused.size == 0:
            return

        # Publish
        stamp = self.get_clock().now().to_msg()
        msg = xyz_to_pointcloud2_numpy(fused, self.target_frame, stamp)
        self.fused_pub.publish(msg)
        self.fusion_count += 1


def main(args=None):
    rclpy.init(args=args)
    node = PointCloudFusionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
