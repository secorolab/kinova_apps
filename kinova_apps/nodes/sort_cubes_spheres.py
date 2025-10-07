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
from scipy.cluster.hierarchy import linkage, fcluster
from sklearn.cluster import KMeans
import cv2
from cv_bridge import CvBridge
from pydantic import BaseModel
from enum import StrEnum

from kinova_apps.utils.pc_utils import cluster_pc, o3dpc_to_ros


class ObjectClass(StrEnum):
    CUBE = "cube"
    BALL = "ball"


class ColorLabel(StrEnum):
    RED = "red"
    GREEN = "green"
    BLUE = "blue"
    YELLOW = "yellow"
    GRAY = "gray"
    UNKNOWN = "unknown"


class ClusterInfo(BaseModel):
    object_class: ObjectClass
    centroid: list[float]
    color: list[float]
    size: int
    diameter: float
    color_label: ColorLabel = ColorLabel.UNKNOWN


class CubeSphereParams(BaseModel):
    outlier_percentiles: tuple = (5, 90)
    fcluster_thresh: float = 1000.0
    z_range_thresh: float = 0.0065


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

def process_clusters_cube_sphere(
    object_cloud: o3d.geometry.PointCloud,
    labels: np.ndarray,
    params: CubeSphereParams = CubeSphereParams(),
) -> list[ClusterInfo]:
    # --- Cluster size filtering ---
    valid_mask = labels >= 0
    valid_labels = labels[valid_mask]
    unique_labels, counts = np.unique(valid_labels, return_counts=True)
    cluster_sizes = dict(zip(unique_labels, counts))

    sizes = np.array(list(cluster_sizes.values()))
    low, high = np.percentile(sizes, params.outlier_percentiles)
    kept_clusters = [lbl for lbl, sz in cluster_sizes.items() if low <= sz <= high]
    # sort by size descending
    kept_clusters.sort(key=lambda x: cluster_sizes[x])

    if not kept_clusters:
        print("[PC Utils] No clusters after filtering by size")
        return []

    # --- Keep only relevant points ---
    keep_mask = np.isin(labels, kept_clusters)
    points = np.asarray(object_cloud.points)[keep_mask]
    colors = np.asarray(object_cloud.colors)[keep_mask]
    labels = labels[keep_mask]

    # --- Hierarchical grouping by cluster size ---
    kept_sizes = np.array([cluster_sizes[lbl] for lbl in kept_clusters]).reshape(-1, 1)
    Z = linkage(kept_sizes, method="ward")
    groups = fcluster(Z, t=params.fcluster_thresh, criterion="distance")

    # Remap groups (smallest → 1)
    group_means = {grp: np.mean([cluster_sizes[lbl] for lbl, g in zip(kept_clusters, groups) if g == grp])
                   for grp in np.unique(groups)}
    sorted_groups = sorted(group_means.items(), key=lambda x: x[1])
    remap = {old: i + 1 for i, (old, _) in enumerate(sorted_groups)}
    label_to_group = {lbl: remap[g] for lbl, g in zip(kept_clusters, groups)}

    # --- Prepare for KMeans on diameters ---
    diameters = []
    for lbl in kept_clusters:
        mask = labels == lbl
        pts = points[mask]
        diameters.append(np.linalg.norm(pts.max(axis=0) - pts.min(axis=0)))
    diameters = np.array(diameters).reshape(-1, 1)

    diam_labels = KMeans(n_clusters=2, random_state=0).fit_predict(diameters)

    sizes = np.array([cluster_sizes[lbl] for lbl in kept_clusters])
    # Remap KMeans labels so that 1 = small, 2 = large
    if sizes[diam_labels == 0].mean() < sizes[diam_labels == 1].mean():
        kmeans_remap = {0: 1, 1: 2}
    else:
        kmeans_remap = {0: 2, 1: 1}
    diam_labels = np.array([kmeans_remap[d] for d in diam_labels])

    # --- Build cluster info in one pass ---
    clusters: list[ClusterInfo] = []
    for i, lbl in enumerate(kept_clusters):
        mask = labels == lbl
        pts = points[mask]
        cols = colors[mask]
        if pts.size == 0:
            continue

        centroid = pts.mean(axis=0)
        color_mean = cols.mean(axis=0)
        z_range = pts[:, 2].ptp()
        if z_range < params.z_range_thresh:
            continue

        cluster_pc = o3d.geometry.PointCloud()
        cluster_pc.points = o3d.utility.Vector3dVector(pts)
        cluster_pc.colors = o3d.utility.Vector3dVector(cols)

        group_id = label_to_group[lbl]
        size_group = "CUBE" if group_id == 1 or diam_labels[i] == 1 else "BALL"
        object_class = ObjectClass[size_group]
        color_label = get_color_label(color_mean.tolist())

        diameter = float(np.linalg.norm(pts.max(axis=0) - pts.min(axis=0)))

        cluster_info = ClusterInfo(
            object_class=object_class,
            centroid=centroid.tolist(),
            color=color_mean.tolist(),
            size=cluster_sizes[lbl],
            diameter=round(diameter, 3),
            color_label=color_label,
        )
        clusters.append(cluster_info)

    return clusters


def get_color_label(rgb: list[float]) -> ColorLabel:
    # Convert [0,1] → [0,255]
    rgb_255 = np.clip(np.array(rgb, dtype=np.float32).reshape(1, 1, 3) * 255, 0, 255).astype(np.uint8)
    hsv = cv2.cvtColor(rgb_255, cv2.COLOR_RGB2HSV)[0, 0]  # [H, S, V]

    h, s, v = hsv
    if v < 40 or s < 50:  # very dark or desaturated → treat as grayish
        return ColorLabel.GRAY

    # Hue ranges (OpenCV hue = 0–179)
    if (h < 15) or (h >= 160):   # red, orange
        return ColorLabel.RED
    elif 15 <= h < 45:           # yellow
        return ColorLabel.YELLOW
    elif 45 <= h < 85:           # green
        return ColorLabel.GREEN
    elif 85 <= h < 160:          # blue, purple
        return ColorLabel.BLUE
    else:
        return ColorLabel.UNKNOWN

def main():
    rclpy.init()
    node = SortObjects()
    
    while rclpy_ok():
        rclpy.spin_once(node)
    
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
