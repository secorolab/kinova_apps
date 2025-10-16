#!/usr/bin/env python3
import rclpy
import rclpy.task
from rclpy.node import Node
from rclpy.action.client import ActionClient, ClientGoalHandle
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

from kinova_moveit_client.action import MoveToCartesianPose, GripperCommand


class ActionFuture(BaseModel):
    send_goal_future: Optional[rclpy.task.Future] = None
    goal_handle: Optional[ClientGoalHandle] = None
    get_result_future: Optional[rclpy.task.Future] = None

class UserData(BaseModel):
    af: ActionFuture = ActionFuture()
    max_sort: int = 3
    num_sorted: int = 0
    sort_data: Optional[list[ClusterInfo]] = []
    target_position: Optional[list[float]] = None
    gripper_open: bool = True
    target_objects: list[ClusterInfo] = []
    move_arm: bool = False
    return_event: Optional[EventID] = None
    picked_objects: list[ClusterInfo] = []


class SortObjects(Node):
    def __init__(self):
        super().__init__('sort_objects')
        self.logger = self.get_logger()
        self.logger.info('SortObjects node started')

    def configure(self):
        self.seg_plane_pub = self.create_publisher(PointCloud2, '/pc_plane', 10)
        self.pc_cluster_pub = self.create_publisher(PointCloud2, '/pc_cluster', 10)
        self.marker_pub = self.create_publisher(MarkerArray, '/detected_objects', 10)

        # action clients
        self.move_to_pose_ac = ActionClient(self, MoveToCartesianPose, 'move_to_cartesian_pose')
        self.gripper_ac = ActionClient(self, GripperCommand, 'gripper_command')

        self.logger.info('SortObjects node configured')

    def detect_objects(self) -> bool:
        ret = False
        pc_msg: Optional[PointCloud2] = None
        # while not ret:
        ret, pc_msg = wait_for_message(node=self, msg_type=PointCloud2, topic='/camera/depth/color/points', time_to_wait=1)
        # if not rclpy_ok():
        #     self.logger.info('Shutdown detected, exiting')
        #     return
        if not ret:
            self.logger.info('Waiting for point cloud message...')
            return False

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
            return False

        self.logger.info(f'Detected {len(clusters)} objects')

        self.seg_plane_pub.publish(o3dpc_to_ros(plane_cloud, pc_header.frame_id, pc_header.stamp))
        self.pc_cluster_pub.publish(o3dpc_to_ros(obj_cluster_cloud, pc_header.frame_id, pc_header.stamp))
        self.publish_markers(clusters, pc_header)

        return True

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

    def get_move_arm_msg(self, position: list[float]):
        goal_msg = MoveToCartesianPose.Goal()
        goal_msg.target_pose.pose.position.x = position[0]
        goal_msg.target_pose.pose.position.y = position[1]
        goal_msg.target_pose.pose.position.z = position[2]
        goal_msg.target_pose.pose.orientation.x = 0.0
        goal_msg.target_pose.pose.orientation.y = 0.0
        goal_msg.target_pose.pose.orientation.z = 0.0
        goal_msg.target_pose.pose.orientation.w = 1.0
        goal_msg.target_pose.header.frame_id = 'base_link' # TODO: check
        goal_msg.target_pose.header.stamp = self.get_clock().now().to_msg()
        
        return goal_msg

    def get_gripper_cmd_msg(self, position: float):
        goal_msg = GripperCommand.Goal()
        goal_msg.gripper_width = position
        return goal_msg

    def execute_action(self, ac: ActionClient, af: ActionFuture, goal_msg):
        '''
        Generic action execution function.
        Call this function repeatedly until it returns True.

        Parameters:
        - ac: ActionClient
        - af: ActionFuture
        - goal_msg: Goal message to send
        '''

        assert ac is not None, "ActionClient is None"
        assert af is not None, "ActionFuture is None"
        assert goal_msg is not None, "Goal message is None"
    
        # check server availability
        if not ac.server_is_ready():
            self.logger.warning('action server not available')
            return False

        # send goal
        if af.send_goal_future is None:
            af.send_goal_future = ac.send_goal_async(goal_msg)
            return False

        # wait for goal to be accepted
        if not af.send_goal_future.done():
            self.logger.info('Waiting for goal to be accepted...')
            return False

        if af.goal_handle is None:
            af.goal_handle = af.send_goal_future.result()
            if not af.goal_handle or not af.goal_handle.accepted:
                self.logger.error("Goal rejected")
                af.send_goal_future = None
                return False

            self.logger.info("Goal accepted")
            af.get_result_future = af.goal_handle.get_result_async()
            af.send_goal_future = None
            return False

        # wait for result
        if af.get_result_future is None:
            self.logger.warning("get_result_future unexpectedly None")
            return False

        if not af.get_result_future.done():
            self.logger.info("Waiting for result...")
            return False

        result = af.get_result_future.result()
        self.logger.info(f'Action result: {result}')
        
        # reset action future for next use
        self.reset_action_future(af)
        return True

    def reset_action_future(self, af: ActionFuture):
        assert af is not None, "ActionFuture is None"

        # cleanup
        af.send_goal_future = None
        af.get_result_future = None
        af.goal_handle = None

    def move_arm(self, af, target_position):
        self.logger.info('Picking object')
        
        assert isinstance(target_position, list) and len(target_position) == 3, "target_position must be a list of 3 floats"

        move_arm_msg = self.get_move_arm_msg(target_position)
        # send goal to arm
        if not self.execute_action(self.move_to_pose_ac, af, move_arm_msg):
            return False

        return True

    def gripper_control(self, af, open: bool):
        val = 0.9 if open else 0.0

        gripper_msg = self.get_gripper_cmd_msg(val)
        # send goal to gripper
        if not self.execute_action(self.gripper_ac, af, gripper_msg):
            return False

        return True

    def place_object(self):
        self.logger.info('Placing object (simulated)')
        
        if random.random() < 0.1:
            self.logger.info('Placing in progress...')
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
        # TODO: select next object to pick
        ud.gripper_open = False # close gripper to pick after moving arm
        produce_event(fsm.event_data, EventID.E_PICK_OBJECT)

    if consume_event(fsm.event_data, EventID.E_PICK_OBJECT_EXIT):
        ud.gripper_open = True # open gripper to place after moving arm
        produce_event(fsm.event_data, EventID.E_PLACE_OBJECT)

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

def move_arm_step(fsm: FSMData, ud: UserData, node: SortObjects):
    if consume_event(fsm.event_data, EventID.E_MOVE_ARM_ENTER):
        node.logger.info(f"Entered state '{StateID(fsm.current_state_index).name}'")
        
    if not node.move_arm(ud.af, ud.target_position):
        return False

    produce_event(fsm.event_data, ud.return_event)
    return True

def gripper_control_step(fsm: FSMData, ud: UserData, node: SortObjects):
    if consume_event(fsm.event_data, EventID.E_GRIPPER_CONTROL_ENTER):
        node.logger.info(f"Entered state '{StateID(fsm.current_state_index).name}'")

    if not node.gripper_control(ud.af, ud.gripper_open):
        return False

    produce_event(fsm.event_data, ud.return_event)
    return True

def pick_object_step(fsm: FSMData, ud: UserData, node: SortObjects):
    if consume_event(fsm.event_data, EventID.E_PICK_OBJECT_ENTER):
        node.logger.info(f"Entered state '{StateID(fsm.current_state_index).name}'")
        
        target_object = ud.target_objects.pop(0)
        ud.picked_objects.push(target_object)
        ud.target_position = target_object.centroid # TODO: z should be monitored
        node.logger.info(f"Picking object at position: {ud.target_position}")
        ud.move_arm = True
        return False

    if ud.move_arm:
        ud.move_arm = False
        ud.return_event = EventID.E_PICK_MOVE_ARM_EXIT
        produce_event(fsm.event_data, EventID.E_PICK_MOVE_ARM)]
        return False

    if consume_event(fsm.event_data, EventID.E_MOVE_ARM_PICK_ENTER):
        ud.move_arm = False
        ud.gripper_open = False # close gripper to pick
        ud.return_event = EventID.E_PICK_GRIPPER_CONTROL_EXIT
        produce_event(fsm.event_data, EventID.E_PICK_GRIPPER_CONTROL)
        return False

    return False

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
