#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.utilities import ok as rclpy_ok
from rclpy.duration import Duration
from rclpy.time import Time

from message_filters import ApproximateTimeSynchronizer, Subscriber

from sensor_msgs.msg import PointCloud2, Image
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker, MarkerArray

from tf2_ros import TransformListener, Buffer
import tf2_geometry_msgs
from tf2_geometry_msgs import do_transform_pose_stamped

import numpy as np
import open3d as o3d

from cv_bridge import CvBridge


from kinova_apps.scripts.pc_utils import cluster_pc, o3dpc_to_ros, process_clusters_cube_sphere
from kinova_apps.scripts.models import ObjectClass, ClusterInfo


class SortObjects(Node):
    def __init__(self):
        super().__init__('sort_objects')

        self.tf2_buffer = Buffer()
        self.tf2_listener = TransformListener(self.tf2_buffer, self)
        
        self.img_sub = Subscriber(self, Image, '/camera/color/image_raw')
        self.pc_sub = Subscriber(self, PointCloud2, '/camera/depth/color/points')

        self.seg_plane_pub = self.create_publisher(PointCloud2, '/pc_plane', 10)
        self.pc_cluster_pub = self.create_publisher(PointCloud2, '/pc_cluster', 10)
        self.marker_pub = self.create_publisher(MarkerArray, '/detected_objects', 10)

        self.ts = ApproximateTimeSynchronizer(
            [self.img_sub, self.pc_sub], queue_size=10, slop=0.1)
        self.ts.registerCallback(self.callback)

        self.bridge = CvBridge()

        self.get_logger().info('SortObjects node started')

    def callback(self, img_msg: Image, pc_msg: PointCloud2):
        self.get_logger().info('Synchronized messages received')
        
        # rgb_img = self.bridge.imgmsg_to_cv2(img_msg, desired_encoding='bgr8')
        
        pc = point_cloud2.read_points_numpy(pc_msg, skip_nans=False, reshape_organized_cloud=True)
        # replace nan with 0
        pc = np.nan_to_num(pc, nan=0.0)
        self.get_logger().info('Received synchronized messages')

        pc_header = pc_msg.header

        # object detection
        plane_cloud, obj_cluster_cloud, clusters = self.detect_pc_objects(pc)
        
        if len(clusters) == 0:
            self.get_logger().info('No objects detected, skipping publishing')
            return

        self.get_logger().info(f'Detected {len(clusters)} objects')

        self.seg_plane_pub.publish(o3dpc_to_ros(plane_cloud, pc_header.frame_id, pc_header.stamp))
        self.pc_cluster_pub.publish(o3dpc_to_ros(obj_cluster_cloud, pc_header.frame_id, pc_header.stamp))
        self.publish_markers(clusters, pc_header)

        # stop subscriptions
        self.destroy_subscription(self.img_sub.sub)
        self.destroy_subscription(self.pc_sub.sub)


    def detect_pc_objects(self, pc: np.ndarray) -> tuple[o3d.geometry.PointCloud, o3d.geometry.PointCloud, list[ClusterInfo]]:
        plane_cloud, obj_cluster_cloud, labels = cluster_pc(pc)
        clusters = process_clusters_cube_sphere(obj_cluster_cloud, labels)

        return plane_cloud, obj_cluster_cloud, clusters


    def publish_markers(self, clusters: list[ClusterInfo], pc_header: Header):
        marker_array = MarkerArray()
        marker_id = 0
        for cluster in clusters:

            centroid = cluster.centroid

            if cluster.object_class == ObjectClass.CUBE:
                object_class = Marker.CUBE
            elif cluster.object_class == ObjectClass.BALL:
                object_class = Marker.SPHERE
            else:
                self.get_logger().warning(f'Unknown object class: {cluster.object_class}')
                continue

            object_scale = cluster.diameter

            marker = Marker()
            marker.ns = "detected_objects"
            marker.id = marker_id
            marker.type = object_class
            marker.action = Marker.ADD
            marker.scale.x = object_scale
            marker.scale.y = object_scale
            marker.scale.z = object_scale

            marker.header = pc_header
            marker.pose.position.x = float(centroid[0])
            marker.pose.position.y = float(centroid[1])
            marker.pose.position.z = float(centroid[2])

            # marker color from cluster color
            color = cluster.color
            marker.color.r = float(color[0])
            marker.color.g = float(color[1])
            marker.color.b = float(color[2])

            marker.color.a = 1.0
            marker.lifetime = Duration(seconds=0).to_msg()
            marker_array.markers.append(marker)
            marker_id += 1

        self.marker_pub.publish(marker_array)


def main():
    rclpy.init()
    node = SortObjects()
    
    while rclpy_ok():
        rclpy.spin_once(node)
    
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
