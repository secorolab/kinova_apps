import numpy as np
import open3d as o3d
from scipy.cluster.hierarchy import linkage, fcluster

from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header

from kinova_apps.scripts.models import ObjectClass, ClusterInfo, ClusterParams, CubeSphereParams, ColorLabel

from sklearn.cluster import KMeans

import cv2


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


def cluster_pc(
    pc: np.ndarray, params: ClusterParams = ClusterParams()
) -> tuple[o3d.geometry.PointCloud, o3d.geometry.PointCloud, np.ndarray]:
    """
    Segment a plane and cluster the remaining objects in a point cloud.

    Returns:
        plane_cloud (PointCloud): the segmented plane
        objects_cloud (PointCloud): clustered objects (filtered & recolored)
    """

    # --- Convert numpy -> Open3D point cloud ---
    o3d_pc = o3d.geometry.PointCloud()
    o3d_pc.points = o3d.utility.Vector3dVector(pc[:, :, :3].reshape(-1, 3))

    # Unpack packed RGB
    rgb_packed = pc[:, :, 3].reshape(-1).view(np.uint32)
    r = ((rgb_packed >> 16) & 255).astype(np.uint8)
    g = ((rgb_packed >> 8) & 255).astype(np.uint8)
    b = (rgb_packed & 255).astype(np.uint8)
    rgb = np.stack([r, g, b], axis=-1).astype(np.float32) / 255.0
    o3d_pc.colors = o3d.utility.Vector3dVector(rgb)

    # --- Downsample ---
    downpcd = o3d_pc.voxel_down_sample(voxel_size=params.voxel_size)

    # --- Plane segmentation ---
    _, inliers = downpcd.segment_plane(
        distance_threshold=params.plane_seg.plane_dist_thresh,
        ransac_n=params.plane_seg.ransac_n,
        num_iterations=params.plane_seg.num_iters,
    )
    plane_cloud = downpcd.select_by_index(inliers)
    object_cloud = downpcd.select_by_index(inliers, invert=True)

    # --- Object clustering (DBSCAN) ---
    labels = np.array(
        object_cloud.cluster_dbscan(
            eps=params.dbscan.eps,
            min_points=params.dbscan.min_points,
            print_progress=False,
        )
    )
    if labels.size == 0:
        print("[PC Utils] No object clusters found")

    return plane_cloud, object_cloud, labels


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
