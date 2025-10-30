#!/usr/bin/env python3
import time
import rclpy
import rclpy.task
from rclpy.node import Node
from rclpy.action.client import ActionClient, ClientGoalHandle
from rclpy.utilities import ok as rclpy_ok
from rclpy.duration import Duration
from rclpy.wait_for_message import wait_for_message

from std_msgs.msg import String
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Pose, PoseArray, PoseStamped

import numpy as np
import open3d as o3d
from pydantic import BaseModel
from typing import Optional
from enum import StrEnum

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
    ColorLabel,
 process_clusters_cube_sphere
)

from kinova_moveit_client.action import MoveToCartesianPose, GripperCommand


class TaskStatus(StrEnum):
    NOT_STARTED = "not_started"
    WAITING = "waiting"
    IN_PROGRESS = "in_progress"
    COMPLETED = "completed"


class TaskControl(StrEnum):
    START = "start"
    STOP = "stop"
    WAIT = "wait"
    CONTINUE = "continue"
    NONE = "none"


class ActionFuture(BaseModel):
    send_goal_future: Optional[rclpy.task.Future] = None
    goal_handle: Optional[ClientGoalHandle] = None
    get_result_future: Optional[rclpy.task.Future] = None

    class Config:
        arbitrary_types_allowed = True

class UserData(BaseModel):
    af: ActionFuture = ActionFuture()
    max_sort: int = 3
    num_sorted: int = 0
    sort_data: list[ClusterInfo] = []
    target_position: list[float] = []
    gripper_open: bool = True
    target_objects: list[ClusterInfo] = []
    pick_object: bool = False
    place_object: bool = False
    detect_objects: bool = False
    current_object: Optional[ClusterInfo] = None

    class Config:
        arbitrary_types_allowed = True


class SortObjects(Node):
    def __init__(self):
        super().__init__('sort_objects')
        self.logger = self.get_logger()
        self.logger.info('SortObjects node started')

    def configure(self):
        # subscribers
        self.exp_control_sub = self.create_subscription(
            String,
            'sorting_task/control',
            self.exp_control_callback,
            10
        )

        # timers
        # self.task_status_timer = self.create_timer(0.01, self.publish_task_status)

        # publishers
        self.task_status_pub = self.create_publisher(String, 'sorting_task/status', 10)
        self.seg_plane_pub = self.create_publisher(PointCloud2, '/pc_plane', 10)
        self.pc_cluster_pub = self.create_publisher(PointCloud2, '/pc_cluster', 10)
        self.marker_pub = self.create_publisher(MarkerArray, '/detected_objects', 10)
        self.obj_pose_pub = self.create_publisher(PoseArray, '/detected_object_poses', 10)
        self.current_obj_pose_pub = self.create_publisher(PoseStamped, '/current_object_pose', 10)

        # action clients
        self.move_to_pose_ac = ActionClient(self, MoveToCartesianPose, 'move_to_cartesian_pose')
        self.gripper_ac = ActionClient(self, GripperCommand, 'gripper_command')

        self.exp_status_data = TaskStatus.NOT_STARTED
        self.exp_control_data = TaskControl.WAIT

        self.logger.info('SortObjects node configured')

    # def publish_task_status(self):
    #     msg = String()
    #     msg.data = self.exp_status_data.value
    #     self.task_status_pub.publish(msg)

    def exp_control_callback(self, msg: String):
        self.exp_control_data = TaskControl(msg.data)
        self.logger.info(f'Received experiment control command: {self.exp_control_data}')

    def detect_objects(self, target_objects: list[ClusterInfo]) -> bool:
        ret = False
        pc_msg: Optional[PointCloud2] = None
 
        ret, pc_msg = wait_for_message(node=self, msg_type=PointCloud2, topic='/camera/depth/color/points', time_to_wait=1)
        if not ret:
            self.logger.info('Waiting for point cloud message...')
            return False

        assert isinstance(pc_msg, PointCloud2)

        pc = point_cloud2.read_points_numpy(pc_msg, skip_nans=False, reshape_organized_cloud=True)
        # replace nan with 0
        pc = np.nan_to_num(pc, nan=0.0)

        pc_header = pc_msg.header
        # print(f"HEADER {pc_header.frame_id}")

        # object detection
        plane_cloud, obj_cluster_cloud, clusters = self.detect_pc_objects(pc)

        # set target objects
        # target_objects.clear()
        # target_objects.extend(clusters)

        # TODO: set the Z coordinate of the centroids to a fixed height for picking here?
        
        if len(clusters) == 0:
            self.logger.info('No objects detected, skipping publishing')
            return False

        self.logger.info(f'Detected {len(clusters)} objects')

        color_groups = {}
        for cluster in clusters:
            color_label = cluster.color_label
            possible_colors = [ColorLabel.RED, ColorLabel.GREEN, ColorLabel.BLUE, ColorLabel.YELLOW]
            if color_label not in color_groups and color_label in possible_colors:
                color_groups[color_label] = []
                color_groups[color_label].append(cluster)
        
        selected_colors = random.sample(list(color_groups.keys()), min(2, len(color_groups)))
        filtered_clusters = []
        for cluster in clusters:
            if cluster.color_label in selected_colors:
                filtered_clusters.append(cluster)
        filtered_clusters = filtered_clusters[:3]

        self.logger.info(f'Selected {len(filtered_clusters)} objects with colors {[cluster.color_label.value for cluster in filtered_clusters]}')

        # set target objects
        target_objects.clear()
        target_objects.extend(filtered_clusters)

        pose_array = PoseArray()
        pose_array.header = pc_header
        pose_array.header.stamp = self.get_clock().now().to_msg()
        for cluster in clusters:
            centroid = cluster.centroid

            pose = Pose()
            pose.position.x = float(centroid[0])
            pose.position.y = float(centroid[1])
            pose.position.z = float(centroid[2])
            pose.orientation.x = 0.0
            pose.orientation.y = 0.0
            pose.orientation.z = 0.0
            pose.orientation.w = 1.0
            pose_array.poses.append(pose)
        
        self.obj_pose_pub.publish(pose_array)

        self.seg_plane_pub.publish(o3dpc_to_ros(plane_cloud, pc_header.frame_id, pc_header.stamp))
        self.pc_cluster_pub.publish(o3dpc_to_ros(obj_cluster_cloud, pc_header.frame_id, pc_header.stamp))
        self.publish_markers(clusters, pc_header)

        time.sleep(1.0)

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

    def publish_current_object_pose(self, centroid: list[float]):
        obj_pose = PoseStamped()
        obj_pose.header.frame_id = 'kinova_camera_color_frame'
        obj_pose.header.stamp = self.get_clock().now().to_msg()
        obj_pose.pose.position.x = float(centroid[0])
        obj_pose.pose.position.y = float(centroid[1])
        obj_pose.pose.position.z = float(centroid[2])
        obj_pose.pose.orientation.x = 0.0
        obj_pose.pose.orientation.y = 0.0
        obj_pose.pose.orientation.z = 0.0
        obj_pose.pose.orientation.w = 1.0

        self.current_obj_pose_pub.publish(obj_pose)

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
        goal_msg.target_pose.pose.position.z = position[2] - 0.025
        goal_msg.target_pose.pose.orientation.x = 0.0
        goal_msg.target_pose.pose.orientation.y = 0.0
        goal_msg.target_pose.pose.orientation.z = 1.0
        goal_msg.target_pose.pose.orientation.w = 0.0
        goal_msg.target_pose.header.frame_id = 'kinova_camera_color_frame'
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

    def move_arm(self, af: ActionFuture, target_position: list[float]):
        self.logger.info('Picking object')
        
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
        ud.sort_data = []
        ud.detect_objects = True
        return False

    if node.exp_control_data == TaskControl.WAIT:
        node.logger.info('Experiment status is wait...', throttle_duration_sec=2.0)
        produce_event(fsm.event_data, EventID.E_WAIT)
        return False

    if ud.num_sorted >= ud.max_sort:
        node.logger.info(f'Sorted {ud.num_sorted} objects, exiting sorting')
        node.exp_status_data = TaskStatus.COMPLETED
        return True

    if ud.detect_objects:
        node.logger.info('Starting object detection for sorting')
        produce_event(fsm.event_data, EventID.E_SORTING_DETECT_OBJECTS)
        return False

    if consume_event(fsm.event_data, EventID.E_DETECT_OBJECTS_SORTING):
        assert len(ud.target_objects) > 0, "No target objects detected for sorting"

        ud.current_object = ud.target_objects.pop()
        ud.gripper_open = False # close gripper to pick after moving arm
        ud.pick_object = True
        ud.place_object = False

        node.publish_current_object_pose(ud.current_object.centroid)
        
        produce_event(fsm.event_data, EventID.E_MOVE_ARM)
        return False

    if consume_event(fsm.event_data, EventID.E_GC_SORTING_ENTER):
        if ud.pick_object:
            node.logger.info(f'Picked object, moving to place position')
            # after picking, move to place position
            ud.gripper_open = True # open gripper to place after moving arm
            ud.place_object = True
            ud.pick_object = False
            produce_event(fsm.event_data, EventID.E_MOVE_ARM)
            return False

        if ud.place_object:
            ud.num_sorted += 1
            assert ud.current_object is not None, "current_object is None"
            ud.sort_data.append(ud.current_object)

            ud.place_object = False
            ud.pick_object = True
            node.logger.info(f'Placed {ud.num_sorted} object.')

            node.exp_control_data = TaskControl.WAIT
            produce_event(fsm.event_data, EventID.E_WAIT)
            return False

    if not ud.place_object and ud.pick_object:
        assert len(ud.target_objects) > 0, "No target objects detected for sorting"

        ud.current_object = ud.target_objects.pop()
        ud.gripper_open = False # close gripper to pick after moving arm
        ud.pick_object = True
        ud.place_object = False

        produce_event(fsm.event_data, EventID.E_MOVE_ARM)
        return False

    return False

def detect_objects_step(fsm: FSMData, ud: UserData, node: SortObjects):
    if consume_event(fsm.event_data, EventID.E_DETECT_OBJECTS_ENTER):
        node.logger.info(f"Entered state '{StateID(fsm.current_state_index).name}'")
        return False

    ud.detect_objects = False
    return node.detect_objects(ud.target_objects)
    
    if random.random() < 0.2:
        return False

    demo_obj = ClusterInfo(
        centroid=[0.5, 0.0, 0.2],
        color=[1.0, 0.0, 0.0],
        diameter=0.05,
        object_class=ObjectClass.CUBE,
        size=1000
    )

    ud.target_objects = [demo_obj for _ in range(ud.max_sort)]
    return True


def move_arm_step(fsm: FSMData, ud: UserData, node: SortObjects):
    if consume_event(fsm.event_data, EventID.E_MOVE_ARM_ENTER):
        node.logger.info(f"Entered state '{StateID(fsm.current_state_index).name}'")
        assert ud.current_object is not None, "current_object is None"
        ud.target_position = ud.current_object.centroid
        print(f"Target position: {ud.target_position}")
        return False
        
    assert len(ud.target_position) == 3, "target_position must be a list of 3 floats"
    if not node.move_arm(ud.af, ud.target_position):
        return False
    
    return True

    return random.random() < 0.8

def gripper_control_step(fsm: FSMData, ud: UserData, node: SortObjects):
    if consume_event(fsm.event_data, EventID.E_GRIPPER_CONTROL_ENTER):
        node.logger.info(f"Entered state '{StateID(fsm.current_state_index).name}'")

    if not node.gripper_control(ud.af, ud.gripper_open):
        return False
    
    return True

    return random.random() < 0.8

def wait_step(fsm: FSMData, ud: UserData, node: SortObjects):
    if consume_event(fsm.event_data, EventID.E_WAIT_ENTER):
        node.logger.info(f"Entered state '{StateID(fsm.current_state_index).name}'")
        if not node.exp_status_data == TaskStatus.NOT_STARTED:
            node.exp_status_data = TaskStatus.WAITING
        return False

    if node.exp_control_data == TaskControl.CONTINUE:
        node.logger.info('Experiment status is continue, resuming sorting')
        node.exp_control_data = TaskControl.NONE
        node.exp_status_data = TaskStatus.IN_PROGRESS
        node.task_status_pub.publish(String(data=node.exp_status_data.value))
        print(f'control_data: {node.exp_control_data}, status_data: {node.exp_status_data}')
        return True

    node.logger.info('Waiting for continue command...', throttle_duration_sec=2.0)
    node.task_status_pub.publish(String(data=node.exp_status_data.value))

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
        StateID.S_MOVE_ARM: {
            "step": move_arm_step,
            "on_end": lambda fsm, ud: generic_on_end(
                fsm, ud, [EventID.E_MOVE_ARM_EXIT]
            ),
        },
        StateID.S_GRIPPER_CONTROL: {
            "step": gripper_control_step,
            "on_end": lambda fsm, ud: generic_on_end(
                fsm, ud, [EventID.E_GRIPPER_CONTROL_EXIT]
            ),
        },
        StateID.S_WAIT: {
            "step": wait_step,
            "on_end": lambda fsm, ud: generic_on_end(
                fsm, ud, [EventID.E_WAIT_EXIT]
            ),
        },
    }

    ud = UserData()

    while rclpy_ok():
        rclpy.spin_once(sort_objects_node, timeout_sec=0.01)

        if fsm.current_state_index == StateID.S_EXIT:
            sort_objects_node.get_logger().info('FSM reached EXIT state, shutting down')
            break
        
        reconfig_event_buffers(fsm.event_data)
        produce_event(fsm.event_data, EventID.E_STEP)
        fsm_behavior(fsm, ud, fsm_bhv, sort_objects_node)
        
        reconfig_event_buffers(fsm.event_data)
        fsm_step(fsm)

    
    sort_objects_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
