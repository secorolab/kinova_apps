from enum import StrEnum
from pydantic import BaseModel


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
    

class CubeSphereParams(BaseModel):
    outlier_percentiles: tuple = (5, 90)
    fcluster_thresh: float = 1000.0
    z_range_thresh: float = 0.0065
