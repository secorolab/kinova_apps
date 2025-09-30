#!/usr/bin/env python3
import rclpy
from rclpy import time
from rclpy.node import Node
from rclpy.utilities import ok as rclpy_ok
from rclpy.duration import Duration

from message_filters import ApproximateTimeSynchronizer, Subscriber

from sensor_msgs.msg import PointCloud2, Image
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header

from visualization_msgs.msg import Marker, MarkerArray

import numpy as np

from cv_bridge import CvBridge

from enum import StrEnum

from kinova_apps.scripts.pc_utils import cluster_pc, o3dpc_to_ros


class OBJECT_CLASS(StrEnum):
    CUBE = "cube"
    BALL = "ball"


class SortObjects(Node):
    def __init__(self):
        super().__init__('sort_objects')
        
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
        rgb_img = self.bridge.imgmsg_to_cv2(img_msg, desired_encoding='bgr8')
        pc = point_cloud2.read_points_numpy(pc_msg, skip_nans=False, reshape_organized_cloud=True)
        # replace nan with 0
        pc = np.nan_to_num(pc, nan=0.0)
        self.get_logger().info('Received synchronized messages')

        pc_header = pc_msg.header

        # object detection
        self.detect_objects(rgb_img, pc, pc_header)

        # stop subscriptions
        self.destroy_subscription(self.img_sub.sub)
        self.destroy_subscription(self.pc_sub.sub)

        exit(0)

    def detect_objects(self, rgb_img: np.ndarray, pc: np.ndarray, pc_header: Header):
        plane_cloud, obj_cluster_cloud, clusters = cluster_pc(pc)

        self.seg_plane_pub.publish(o3dpc_to_ros(plane_cloud, pc_header.frame_id, pc_header.stamp))
        self.pc_cluster_pub.publish(o3dpc_to_ros(obj_cluster_cloud, pc_header.frame_id, pc_header.stamp))

        marker_array = MarkerArray()
        marker_id = 0
        for cluster in clusters:
            # Compute centroid
            centroid = cluster["centroid"]

            if cluster["group"] == 1:
                object_class = Marker.CUBE
                object_scale = 0.03
            elif cluster["group"] == 2:
                object_class = Marker.SPHERE
                object_scale = 0.05
            else:
                object_class = Marker.CYLINDER
                object_scale = 0.1

            # Create a marker for the centroid
            marker = Marker()
            marker.header.frame_id = pc_header.frame_id
            marker.header.stamp = pc_header.stamp
            marker.ns = "detected_objects"
            marker.id = marker_id
            marker.type = object_class
            marker.action = Marker.ADD
            marker.pose.position.x = float(centroid[0])
            marker.pose.position.y = float(centroid[1])
            marker.pose.position.z = float(centroid[2])
            marker.pose.orientation.x = 0.0
            marker.pose.orientation.y = 0.0
            marker.pose.orientation.z = 0.0
            marker.pose.orientation.w = 1.0
            marker.scale.x = object_scale
            marker.scale.y = object_scale
            marker.scale.z = object_scale

            # marker color from cluster color
            color = cluster["color"]
            marker.color.r = float(color[0])
            marker.color.g = float(color[1])
            marker.color.b = float(color[2])

            marker.color.a = 1.0
            marker.lifetime = Duration(seconds=0).to_msg()
            marker_array.markers.append(marker)
            marker_id += 1

        self.marker_pub.publish(marker_array)
        self.get_logger().info(f'Published {len(marker_array.markers)} detected object markers')


def main():
    rclpy.init()
    node = SortObjects()
    
    while rclpy_ok():
        rclpy.spin_once(node)
    
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
