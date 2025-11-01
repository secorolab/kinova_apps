import numpy as np
import open3d as o3d
from scipy.cluster.hierarchy import linkage, fcluster
from sklearn.cluster import KMeans
import cv2
from pydantic import BaseModel
from enum import StrEnum


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
    world_position: list[float]


class CubeSphereParams(BaseModel):
    outlier_percentiles: tuple = (2, 97)
    fcluster_thresh: float = 1000.0
    z_range_thresh: float = 0.0065


def process_clusters_cube_sphere(
    object_cloud: o3d.geometry.PointCloud,
    labels: np.ndarray,
    params: CubeSphereParams = CubeSphereParams()) -> list[ClusterInfo]:
    
    # --- Cluster size filtering ---
    valid_mask = labels >= 0
    valid_labels = labels[valid_mask]
    unique_labels, counts = np.unique(valid_labels, return_counts=True)
    cluster_sizes = dict(zip(unique_labels, counts))

    sizes = np.array(list(cluster_sizes.values()))
    _, high = np.percentile(sizes, params.outlier_percentiles)
    low = 100
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
    print(f"[PC Utils] Cluster sizes: {len(cluster_sizes)}: {dict(sorted(cluster_sizes.items(), key=lambda x: x[1]))}")
    print(f"[PC Utils] Kept sizes:    {len(kept_sizes)}: {kept_sizes.flatten().tolist()}")

    cluster_z_mins = {}
    for lbl in kept_clusters:
        mask = labels == lbl
        pts = points[mask]
        z_min = pts[:, 2].min()
        cluster_z_mins[lbl] = z_min

    # for lbl in sorted(kept_clusters, key=lambda x: cluster_sizes[x]):
    #     size = cluster_sizes[lbl]
    #     z_min = cluster_z_mins[lbl]
    #     print(f"Cluster {lbl} size={size}, z_min={z_min}")

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
        z_range = np.ptp(pts[:, 2])
        if z_range < params.z_range_thresh:
            continue

        cluster_pc = o3d.geometry.PointCloud()
        cluster_pc.points = o3d.utility.Vector3dVector(pts)
        cluster_pc.colors = o3d.utility.Vector3dVector(cols)

        group_id = label_to_group[lbl]
        size_group = "CUBE" if group_id == 1 else "BALL"
        object_class = ObjectClass[size_group]
        color_label = get_color_label(color_mean.tolist())

        diameter = float(np.linalg.norm(pts.max(axis=0) - pts.min(axis=0)))

        cluster_info = ClusterInfo(
            object_class=object_class,
            centroid=centroid.tolist(),
            world_position=[],
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


