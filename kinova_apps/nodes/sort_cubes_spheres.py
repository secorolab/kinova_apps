#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.utilities import ok as rclpy_ok
from rclpy.duration import Duration
from rclpy.wait_for_message import wait_for_message

from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header
from visualization_msgs.msg import Marker, MarkerArray

import numpy as np
import open3d as o3d
from pydantic import BaseModel
from typing import Optional

import signal
import sys
import random

from coord_dsl.event_loop import (
    produce_event,
    consume_event,
    reconfig_event_buffers,
)
from coord_dsl.fsm import FSMData, fsm_step

from kinova_apps.scripts.fsm_kinova_sorting import (
    EventID,
    StateID,
    create_fsm
)
from kinova_apps.utils.pc_utils import cluster_pc, o3dpc_to_ros
from kinova_apps.utils.detect_cubes_spheres_from_pc import (
    ObjectClass,
    ClusterInfo,
    process_clusters_cube_sphere
)


class UserData(BaseModel):
    max_sort: int = 3
    num_sorted: int = 0
    sort_data: Optional[list[ClusterInfo]] = []


class SortObjects(Node):
    def __init__(self):
        super().__init__('sort_objects')
        self.logger = self.get_logger()
        self.logger.info('SortObjects node started')

    def configure(self):
        self.seg_plane_pub = self.create_publisher(PointCloud2, '/pc_plane', 10)
        self.pc_cluster_pub = self.create_publisher(PointCloud2, '/pc_cluster', 10)
        self.marker_pub = self.create_publisher(MarkerArray, '/detected_objects', 10)
        self.logger.info('SortObjects node configured')

    def detect(self):
        ret = False
        pc_msg: Optional[PointCloud2] = None
        while not ret:
            ret, pc_msg = wait_for_message(node=self, msg_type=PointCloud2, topic='/camera/depth/color/points')
            if not rclpy_ok():
                self.logger.info('Shutdown detected, exiting')
                return
            if not ret:
                self.logger.info('Waiting for point cloud message...')

        assert isinstance(pc_msg, PointCloud2)

        pc = point_cloud2.read_points_numpy(pc_msg, skip_nans=False, reshape_organized_cloud=True)
        # replace nan with 0
        pc = np.nan_to_num(pc, nan=0.0)
        self.logger.info('Received synchronized messages')

        pc_header = pc_msg.header

        # object detection
        plane_cloud, obj_cluster_cloud, clusters = self.detect_pc_objects(pc)
        
        if len(clusters) == 0:
            self.logger.info('No objects detected, skipping publishing')
            return

        self.logger.info(f'Detected {len(clusters)} objects')

        self.seg_plane_pub.publish(o3dpc_to_ros(plane_cloud, pc_header.frame_id, pc_header.stamp))
        self.pc_cluster_pub.publish(o3dpc_to_ros(obj_cluster_cloud, pc_header.frame_id, pc_header.stamp))
        self.publish_markers(clusters, pc_header)

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
                self.logger.warning(f'Unknown object class: {cluster.object_class}')
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

    def home_arm(self):
        self.logger.info('Homing arm (simulated)')
        
        if random.random() < 0.1:
            self.logger.info('Homing in progress...')
            return False

        return True

    def pick_object(self):
        self.logger.info('Picking object (simulated)')
        
        if random.random() < 0.1:
            self.logger.info('Picking in progress...')
            return False

        return True

    def place_object(self):
        self.logger.info('Placing object (simulated)')
        
        if random.random() < 0.1:
            self.logger.info('Placing in progress...')
            return False

        return True

    def detect_objects(self):
        self.logger.info('Detecting objects...')
        
        if random.random() < 0.1:
            self.logger.info('Detection in progress...')
            return False

        return True



def fsm_behavior(fsm: FSMData, ud: UserData, bhv_data: dict[StateID, dict], node: SortObjects):
    cs = fsm.current_state_index
    if cs not in bhv_data:
        return

    bhv_data_cs = bhv_data[StateID(cs)]

    assert "step" in bhv_data_cs, f"no step defined for state: {cs}"
    if not bhv_data_cs["step"](fsm, ud, node):
        return

    if "on_end" in bhv_data_cs:
        bhv_data_cs["on_end"](fsm, ud)


def signal_handler(sig, frame):
    print("You pressed Ctrl+C! Exiting gracefully...")
    rclpy.shutdown()
    sys.exit(0)


def generic_on_end(fsm: FSMData, ud: UserData, end_events: list[EventID]):
    print(f"State '{StateID(fsm.current_state_index).name}' finished")
    for evt in end_events:
        produce_event(fsm.event_data, evt)


def configure_step(fsm: FSMData, ud: UserData, node: SortObjects):
    if consume_event(fsm.event_data, EventID.E_CONFIGURE_ENTER):
        node.logger.info(f"Entered state '{StateID(fsm.current_state_index).name}'")
        
        node.configure()

        return True
    
    return False


def idle_step(fsm: FSMData, ud: UserData, node: SortObjects):
    if consume_event(fsm.event_data, EventID.E_IDLE_ENTER):
        node.logger.info(f"Entered state '{StateID(fsm.current_state_index).name}'")
        return True

    if consume_event(fsm.event_data, EventID.E_SORTING_EXIT_IDLE_ENTER):
        node.logger.info("Returned to idle from sorting")
        produce_event(fsm.event_data, EventID.E_IDLE_EXIT)
        return False

    if consume_event(fsm.event_data, EventID.E_CONFIGURE_IDLE):
        node.logger.info("Entered idle from configure")
        return True

    produce_event(fsm.event_data, EventID.E_IDLE_HOME_ARM)

    return False

def home_arm_step(fsm: FSMData, ud: UserData, node: SortObjects):
    if consume_event(fsm.event_data, EventID.E_HOME_ARM_ENTER):
        node.logger.info(f"Entered state '{StateID(fsm.current_state_index).name}'")
        
    return node.home_arm()

def sorting_step(fsm: FSMData, ud: UserData, node: SortObjects):
    if consume_event(fsm.event_data, EventID.E_SORTING_ENTER):
        node.logger.info(f"Entered state '{StateID(fsm.current_state_index).name}'")
        ud.num_sorted = 0
        node.logger.info('Detecting objects for the first time')
        produce_event(fsm.event_data, EventID.E_SORTING_DETECT_OBJECTS)
        return False

    if ud.num_sorted >= ud.max_sort:
        node.logger.info(f'Sorted {ud.num_sorted} objects, exiting sorting')
        return True

    if consume_event(fsm.event_data, EventID.E_DETECT_OBJECTS_SORTING):
        node.logger.info(f"Returned to sorting from detect objects, sorted {ud.num_sorted} objects")
        produce_event(fsm.event_data, EventID.E_PICK_OBJECT)

    if consume_event(fsm.event_data, EventID.E_PLACE_OBJECT_SORTING):
        ud.num_sorted += 1
        node.logger.info(f"Returned to sorting from place object, sorted {ud.num_sorted} objects")
        if ud.num_sorted < ud.max_sort:
            produce_event(fsm.event_data, EventID.E_SORTING_DETECT_OBJECTS)

    return False

def detect_objects_step(fsm: FSMData, ud: UserData, node: SortObjects):
    if consume_event(fsm.event_data, EventID.E_DETECT_OBJECTS_ENTER):
        node.logger.info(f"Entered state '{StateID(fsm.current_state_index).name}'")

    return node.detect_objects()

def pick_object_step(fsm: FSMData, ud: UserData, node: SortObjects):
    if consume_event(fsm.event_data, EventID.E_PICK_OBJECT_ENTER):
        node.logger.info(f"Entered state '{StateID(fsm.current_state_index).name}'")

    return node.pick_object()

def place_object_step(fsm: FSMData, ud: UserData, node: SortObjects):
    if consume_event(fsm.event_data, EventID.E_PLACE_OBJECT_ENTER):
        node.logger.info(f"Entered state '{StateID(fsm.current_state_index).name}'")

    return node.place_object()


def main():
    rclpy.init()
    signal.signal(signal.SIGINT, signal_handler)
    
    sort_objects_node = SortObjects()
    
    fsm = create_fsm()

    fsm_bhv = {
        StateID.S_CONFIGURE: {
            "step": configure_step,
            "on_end": lambda fsm, ud: generic_on_end(
                fsm, ud, [EventID.E_CONFIGURE_EXIT]
            ),
        },
        StateID.S_IDLE: {
           "step": idle_step,
            "on_end": lambda fsm, ud: generic_on_end(
                fsm, ud, [EventID.E_STEP]
            ),
        },
        StateID.S_HOME_ARM: {
            "step": home_arm_step,
            "on_end": lambda fsm, ud: generic_on_end(
                fsm, ud, [EventID.E_HOME_ARM_EXIT]
            ),
        },
        StateID.S_SORTING: {
            "step": sorting_step,
            "on_end": lambda fsm, ud: generic_on_end(
                fsm, ud, [EventID.E_SORTING_EXIT]
            ),
        },
        StateID.S_DETECT_OBJECTS: {
            "step": detect_objects_step,
            "on_end": lambda fsm, ud: generic_on_end(
                fsm, ud, [EventID.E_DETECT_OBJECTS_EXIT]
            ),
        },
        StateID.S_PICK_OBJECT: {
            "step": pick_object_step,
            "on_end": lambda fsm, ud: generic_on_end(
                fsm, ud, [EventID.E_PICK_OBJECT_EXIT]
            ),
        },
        StateID.S_PLACE_OBJECT: {
            "step": place_object_step,
            "on_end": lambda fsm, ud: generic_on_end(
                fsm, ud, [EventID.E_PLACE_OBJECT_EXIT]
            ),
        },
    }

    ud = UserData()

    while rclpy_ok():
        # rclpy.spin_once(sort_objects_node, timeout_sec=0.1)

        if fsm.current_state_index == StateID.S_EXIT:
            sort_objects_node.get_logger().info('FSM reached EXIT state, shutting down')
            break
        
        produce_event(fsm.event_data, EventID.E_STEP)
        fsm_behavior(fsm, ud, fsm_bhv, sort_objects_node)
        reconfig_event_buffers(fsm.event_data)
        
        fsm_step(fsm)
        reconfig_event_buffers(fsm.event_data)

    
    sort_objects_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
