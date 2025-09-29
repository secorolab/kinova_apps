#!/usr/bin/env python3
import rclpy
from rclpy import time
from rclpy.node import Node
from rclpy.utilities import ok as rclpy_ok

import numpy as np
import open3d as o3d
import cv2
from cv_bridge import CvBridge

from sensor_msgs.msg import PointCloud2, Image
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header

from message_filters import ApproximateTimeSynchronizer, Subscriber


def o3dpc_to_ros(pc: o3d.geometry.PointCloud, frame_id: str, stamp=None) -> PointCloud2:
    """Convert Open3D PointCloud to ROS2 PointCloud2 with XYZRGB"""
    # Extract points
    points = np.asarray(pc.points)

    # Extract colors if available, otherwise set white
    if pc.has_colors():
        colors = (np.asarray(pc.colors) * 255).astype(np.uint8)
    else:
        colors = np.ones((points.shape[0], 3), dtype=np.uint8) * 255

    # Pack RGB into a single float32 (same as ROS PointCloud2 expects)
    rgb_packed = (
        (colors[:, 0].astype(np.uint32) << 16) |
        (colors[:, 1].astype(np.uint32) << 8) |
        (colors[:, 2].astype(np.uint32))
    ).astype(np.uint32)

    rgb_packed = rgb_packed.view(np.float32)  # reinterpret as float32

    # Combine XYZ + RGB
    points_with_rgb = np.hstack([points, rgb_packed.reshape(-1, 1)])

    # Build header
    header = Header()
    header.stamp = stamp
    header.frame_id = frame_id

    # Define fields: x,y,z,rgb
    fields = [
        point_cloud2.PointField(name='x', offset=0, datatype=point_cloud2.PointField.FLOAT32, count=1),
        point_cloud2.PointField(name='y', offset=4, datatype=point_cloud2.PointField.FLOAT32, count=1),
        point_cloud2.PointField(name='z', offset=8, datatype=point_cloud2.PointField.FLOAT32, count=1),
        point_cloud2.PointField(name='rgb', offset=12, datatype=point_cloud2.PointField.FLOAT32, count=1),
    ]

    # Create PointCloud2
    pc2_msg = point_cloud2.create_cloud(header, fields, points_with_rgb)
    return pc2_msg


class PCSegmentation(Node):
    def __init__(self):
        super().__init__('pc_segmentation_node')
        
        self.pc_sub = Subscriber(self, PointCloud2, '/camera/depth/color/points')
        self.img_sub = Subscriber(self, Image, '/camera/color/image_raw')

        self.seg_plane_pub = self.create_publisher(PointCloud2, '/pc_plane', 10)
        self.pc_cluster_pub = self.create_publisher(PointCloud2, '/pc_cluster', 10)

        self.ts = ApproximateTimeSynchronizer([self.pc_sub, self.img_sub], queue_size=10, slop=0.1)
        self.ts.registerCallback(self.callback)
    
        self.bridge = CvBridge()

        self.rgb_image = None
        self.pc = None

    def callback(self, pc_msg: PointCloud2, img_msg: Image):
        self.get_logger().info('Received synchronized messages')

        self.rgb_image = self.bridge.imgmsg_to_cv2(img_msg, desired_encoding='bgr8')

        # Convert PointCloud2 to a list of points
        pc_points = point_cloud2.read_points_numpy(pc_msg, skip_nans=False, reshape_organized_cloud=True)
        # replace nan with 0
        pc_points = np.nan_to_num(pc_points, nan=0.0)

        print(f'PC shape: {pc_points.shape}') # (640, 480, 4) - x,y,z,rgb

        self.pc = pc_points

        # convert to open3d point cloud
        o3d_pc = o3d.geometry.PointCloud()
        o3d_pc.points = o3d.utility.Vector3dVector(pc_points[:, :, :3].reshape(-1, 3))
        
        rgb_packed = pc_points[:, :, 3].reshape(-1).view(np.uint32)

        # Unpack into R, G, B
        r = ((rgb_packed >> 16) & 255).astype(np.uint8)
        g = ((rgb_packed >> 8) & 255).astype(np.uint8)
        b = (rgb_packed & 255).astype(np.uint8)

        # Normalize to [0,1] for Open3D
        rgb = np.stack([r, g, b], axis=-1).astype(np.float32) / 255.0

        o3d_pc.colors = o3d.utility.Vector3dVector(rgb)

        # Downsample the point cloud
        downpcd = o3d_pc.voxel_down_sample(voxel_size=0.01)

        # Plane segmentation
        plane_model, inliers = downpcd.segment_plane(distance_threshold=0.005, ransac_n=3, num_iterations=1000)

        # detect object clusters
        inlier_cloud = downpcd.select_by_index(inliers)
        outlier_cloud = downpcd.select_by_index(inliers, invert=True)

        # publish plane to ros
        plane_msg = o3dpc_to_ros(inlier_cloud, frame_id=pc_msg.header.frame_id, stamp=pc_msg.header.stamp)
        self.seg_plane_pub.publish(plane_msg)

        # publish clusters to ros
        labels = np.array(outlier_cloud.cluster_dbscan(eps=0.02, min_points=10, print_progress=False))
        max_label = labels.max()
        
        cluster_msg = o3dpc_to_ros(outlier_cloud, frame_id=pc_msg.header.frame_id, stamp=pc_msg.header.stamp)
        self.pc_cluster_pub.publish(cluster_msg)

        # stop subscribing after first callback
        exit(1)
        

def main(args=None):
    rclpy.init(args=args)
    pc_segmentation_node = PCSegmentation()
    
    while rclpy_ok():
        rclpy.spin_once(pc_segmentation_node)

if __name__ == "__main__":
    main()
