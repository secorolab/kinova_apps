#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

import numpy as np
import open3d as o3d

from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs_py import point_cloud2


def pointcloud2_to_xyzrgb(msg):
    """Convert ROS2 PointCloud2 to Nx6 numpy array (x,y,z,r,g,b)."""
    points = []
    for p in point_cloud2.read_points(msg, skip_nans=True,
                                      field_names=("x", "y", "z", "rgb")):
        x, y, z, rgb = p
        # unpack rgb (float32 -> 3 uchar)
        rgb_int = int(rgb)
        r = (rgb_int >> 16) & 255
        g = (rgb_int >> 8) & 255
        b = rgb_int & 255
        points.append([x, y, z, r, g, b])
    return np.array(points, dtype=np.float32)


def numpy_to_pointcloud2(points, frame_id="map"):
    """Convert Nx6 numpy (x,y,z,r,g,b) to PointCloud2."""
    fields = [
        PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
        PointField(name="r", offset=12, datatype=PointField.FLOAT32, count=1),
        PointField(name="g", offset=16, datatype=PointField.FLOAT32, count=1),
        PointField(name="b", offset=20, datatype=PointField.FLOAT32, count=1),
    ]
    return point_cloud2.create_cloud(msg_header=rclpy.time.Time().to_msg(),
                                     fields=fields,
                                     points=points.tolist())


class TableSegmentationNode(Node):
    def __init__(self):
        super().__init__('table_segmentation')
        self.subscription = self.create_subscription(
            PointCloud2,
            '/camera/depth/points',
            self.callback,
            10)
        self.publisher_clusters = self.create_publisher(PointCloud2,
                                                        '/clusters',
                                                        10)

    def callback(self, msg):
        points = pointcloud2_to_xyzrgb(msg)
        if points.shape[0] < 1000:
            self.get_logger().warn("Not enough points.")
            return

        # Convert to open3d point cloud
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(points[:, :3])
        colors = points[:, 3:] / 255.0
        pcd.colors = o3d.utility.Vector3dVector(colors)

        # Plane segmentation
        plane_model, inliers = pcd.segment_plane(distance_threshold=0.01,
                                                 ransac_n=3,
                                                 num_iterations=1000)
        table_cloud = pcd.select_by_index(inliers)
        objects_cloud = pcd.select_by_index(inliers, invert=True)

        # Clustering
        labels = np.array(objects_cloud.cluster_dbscan(eps=0.02,
                                                       min_points=50,
                                                       print_progress=False))
        n_clusters = labels.max() + 1
        self.get_logger().info(f"Found {n_clusters} clusters")

        for i in range(n_clusters):
            cluster = objects_cloud.select_by_index(
                np.where(labels == i)[0])
            cluster_np = np.hstack([
                np.asarray(cluster.points),
                np.asarray(cluster.colors) * 255.0
            ])
            out_msg = numpy_to_pointcloud2(cluster_np, frame_id=msg.header.frame_id)
            self.publisher_clusters.publish(out_msg)


def main(args=None):
    rclpy.init(args=args)
    node = TableSegmentationNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

