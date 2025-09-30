import numpy as np
import open3d as o3d
import cv2
from cv_bridge import CvBridge

from sklearn.cluster import KMeans
from scipy.cluster.hierarchy import linkage, fcluster

from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header

from pydantic import BaseModel


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


class PlaneSegParams(BaseModel):
    plane_dist_thresh: float = 0.01
    ransac_n: int = 5
    num_iters: int = 1000


class DBSCANParams(BaseModel):
    eps: float = 0.02
    min_points: int = 10


class ClusterParams(BaseModel):
    voxel_size: float = 0.001
    plane_seg: PlaneSegParams = PlaneSegParams()
    dbscan: DBSCANParams = DBSCANParams()
    outlier_percentiles: tuple = (5, 90)
    fcluster_thresh: float = 1000.0


def cluster_pc(
    pc: np.ndarray, params: ClusterParams = ClusterParams()
) -> tuple[o3d.geometry.PointCloud, o3d.geometry.PointCloud, list[dict]]:
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
        print("[PC Utils] No clusters found")
        return None

    # --- Cluster size filtering ---
    unique_labels, counts = np.unique(labels[labels >= 0], return_counts=True)
    cluster_sizes = dict(zip(unique_labels, counts))
    sorted_clusters = sorted(cluster_sizes.items(), key=lambda x: x[1])  # (label, size)

    sizes = [size for _, size in sorted_clusters]
    low, high = np.percentile(sizes, params.outlier_percentiles)
    kept_clusters = [label for label, size in sorted_clusters if low <= size <= high]

    if not kept_clusters:
        print("[PC Utils] No clusters after filtering by size")
        return None

    # --- Keep only points from filtered clusters ---
    keep_indices = np.where(np.isin(labels, kept_clusters))[0]
    object_cloud = object_cloud.select_by_index(keep_indices)
    labels = labels[keep_indices]
    obj_colors = np.asarray(object_cloud.colors)

    # --- Hierarchical grouping ---
    kept_sizes = np.array([cluster_sizes[lbl] for lbl in kept_clusters]).reshape(-1, 1)
    Z = linkage(kept_sizes, method="ward")
    groups = fcluster(Z, t=params.fcluster_thresh, criterion="distance")

    # Build mapping label -> raw group
    label_to_group = {lbl: grp for lbl, grp in zip(kept_clusters, groups)}

    # --- Remap groups so that the smallest clusters = group 1 ---
    # Compute mean size per group
    group_means = {}
    for lbl, grp in label_to_group.items():
        group_means.setdefault(grp, []).append(cluster_sizes[lbl])
    group_means = {grp: np.mean(sizes) for grp, sizes in group_means.items()}

    # Sort groups by mean size, assign new ids
    sorted_groups = sorted(group_means.items(), key=lambda x: x[1])  # smallest → largest
    remap = {old_grp: new_id+1 for new_id, (old_grp, _) in enumerate(sorted_groups)}

    # Apply remap
    label_to_group = {lbl: remap[grp] for lbl, grp in label_to_group.items()}

    # --- Recolor each cluster uniformly ---
    clusters = []
    for lbl, grp in label_to_group.items():
        mask = labels == lbl
        if not np.any(mask):
            continue
        
        cluster_color = obj_colors[mask].mean(axis=0)
        obj_colors[mask] = cluster_color

        centroid = np.mean(np.asarray(object_cloud.points)[mask], axis=0)
        clusters.append({
            "group": grp,  # now guaranteed: smallest = group 1
            "centroid": centroid,
            "color": cluster_color,
            "size": cluster_sizes[lbl],
        })

    # Debug print
    sizes_debug = [c["size"] for c in clusters]
    print(f"[PC Utils] Kept clusters sizes: {sizes_debug}")
    for cluster in clusters:
        print(f"[PC Utils] Cluster group {cluster['group']} - size {cluster['size']}")


    object_cloud.colors = o3d.utility.Vector3dVector(obj_colors)

    return plane_cloud, object_cloud, clusters


