from __future__ import print_function, division
import sys
from copy import copy, deepcopy
import math
import itertools
import numpy as np
import rospy
import tf.transformations as tfm
import unittest
import roslaunch
import time
from xyz_depalletize_motion import utils
from xyz_depalletize_motion.pallet_box import PalletBox
from xyz_depalletize_motion.box_graph import BoxGraph, merge_box_greedily
from xyz_env_manager.msg import Tool, Tip, TipPrimitive, RegionPrimitive, PrimitiveGroup, GeometricPrimitive, IO
from xyz_env_manager.msg import Pose
from xyz_motion import SE3, pose_to_list
import shapely.geometry as sg
import json
import warnings

begin_color = '\033[1;33m'
end_color = '\033[0m'
try:
    with open('tp_io_config.json', 'r') as f:
        tp_io_config = json.load(f)
except FileNotFoundError as err:
    tp_io_config = None

def bulid_tip_primitive(id, tf_tool_region, dimension, max_suction_force, tool_name = None, tp_io_config = tp_io_config):
    '''
    Inputs Data:

    id(string):tip_primitive的id

    tf_tool_region(list or tuple): 其中的吸取suction_region相对于Tool的pose。

    dimensions(list):tip_primitive的尺寸; [c_l, c_w]

    max_suction_force(float):单位吸取重量;

    tp_io_config(dict):在tp_io_config.json文件中定义好的的吸盘io
    '''
    # tip_primitive包括矩形的最小吸取单元，这里函数默认只包含一个最小吸取单元
    # tip_primitive包括矩形的最小吸取单元长一定大于宽，长沿着x轴，tf_tool_region也（tip_primitive相对于Tool原点的pose）以此为标准判断
    assert dimension[0] >= dimension[1] 
    tip_primitive = TipPrimitive()
    tip_primitive.id = id
    tip_suction_region = RegionPrimitive()
    tip_suction_region.type = RegionPrimitive.RECTANGLE
    origin_pose = tf_tool_region
    tip_suction_region.origin.x = origin_pose[0]
    tip_suction_region.origin.y = origin_pose[1]
    tip_suction_region.origin.z = origin_pose[2]
    tip_suction_region.origin.qx = origin_pose[3] 
    tip_suction_region.origin.qy = origin_pose[4]
    tip_suction_region.origin.qz = origin_pose[5] 
    tip_suction_region.origin.qw = origin_pose[6] 
    tip_suction_region.dimensions.extend([dimension[0], dimension[1]])
    tip_primitive.suction_areas.append(tip_suction_region)
    tip_primitive.max_suction_force = max_suction_force
    
    if tp_io_config == None:
        print(begin_color + "未找到IO配置文件，将忽略IO配置" + end_color)
        return tip_primitive

    if tool_name == None:
        print(begin_color + "请检查环境脚本文件是否为最新，bulid_tip_primitive函数需要参数'tool_name'，本次将忽略配置" + end_color)
        return tip_primitive

    if tool_name not in tp_io_config.keys():
        print(begin_color + f"请检查配置文件结构是否为最新，是否包含'tool_name'" + end_color)
        print(begin_color + f"当前tool <{tool_name}> 不在配置文件中，请检查配置文件，本次将忽略配置" + end_color)
        return tip_primitive

    if id not in tp_io_config[tool_name].keys():
        print(begin_color + f"tip_primitive <{id}> 不在配置文件中，请检查配置文件，本次将忽略配置" + end_color)
        return tip_primitive
    tp_io_config = tp_io_config[tool_name][id]
    
    if tp_io_config["pick_ios"]["enable"]:
        device_type = tp_io_config["pick_ios"]["device_type"]
        assert device_type in ["ROBOT", "EXTERN_DEVICE"]
        device_id = tp_io_config["pick_ios"]["device_id"]
        pick_io_ports = tp_io_config["pick_ios"]["io_ports"]
        pick_io_values = tp_io_config["pick_ios"]["io_values"]
        pick_io_interval = tp_io_config["pick_ios"]["io_interval"]
        assert len(pick_io_ports) == len(pick_io_values) == len(pick_io_interval)
        pick_io_list = []
        for i in range(len(pick_io_ports)):
            io = IO()
            io.device_type = IO.ROBOT if device_type == "ROBOT" else IO.EXTERN_DEVICE
            io.device_id = str(device_id)
            io.port_id = pick_io_ports[i]
            pick_io_list.append(io)
        tip_primitive.pick_io_list = pick_io_list
        tip_primitive.pick_io_values = pick_io_values
        tip_primitive.pick_io_interval = pick_io_interval

    if tp_io_config["place_ios"]["enable"]:
        device_type = tp_io_config["place_ios"]["device_type"]
        assert device_type in ["ROBOT", "EXTERN_DEVICE"]
        device_id = tp_io_config["place_ios"]["device_id"]
        place_io_ports = tp_io_config["place_ios"]["io_ports"]
        place_io_values = tp_io_config["place_ios"]["io_values"]
        place_io_interval = tp_io_config["place_ios"]["io_interval"]
        assert len(place_io_ports) == len(place_io_values) == len(place_io_interval)
        place_io_list = []
        for i in range(len(place_io_ports)):
            io = IO()
            io.device_type = IO.ROBOT if device_type == "ROBOT" else IO.EXTERN_DEVICE
            io.device_id = str(device_id)
            io.port_id = place_io_ports[i]
            place_io_list.append(io)
        tip_primitive.place_io_list = place_io_list
        tip_primitive.place_io_values = place_io_values
        tip_primitive.place_io_interval = place_io_interval

    if tp_io_config["check_ios"]["enable"]:
        device_type = tp_io_config["check_ios"]["device_type"]
        assert device_type in ["ROBOT", "EXTERN_DEVICE"]
        device_id = tp_io_config["check_ios"]["device_id"]
        check_io_ports = tp_io_config["check_ios"]["io_ports"]
        check_io_values = tp_io_config["check_ios"]["io_values"]
        check_io_position = tp_io_config["check_ios"]["io_position"]
        assert len(check_io_ports) == len(check_io_values) == len(check_io_position)
        place_io_list = []
        for i in range(len(check_io_ports)):
            io = IO()
            io.device_type = IO.ROBOT if device_type == "ROBOT" else IO.EXTERN_DEVICE
            io.device_id = str(device_id)
            io.port_id = int(check_io_ports[i])
            io.position.point = check_io_position[i]
            io.expected_value = check_io_values[i]
            io.id = str(check_io_ports[i])
            tip_primitive.check_ios.append(io)

    return tip_primitive

def cal_tip_primitives_combination(total_tip_dimension, tf_tool_tip, max_suction_force, partition = [1, 1]):
    '''
    自动计算吸盘分区后的tip_primitives和组合
    Inputs Data:

    total_tip_dimension(list):tip_primitive的尺寸; [Length, Width]， Length一定大于等于Width
    
    tf_tool_tip: tip相对于Tool自身原点的pose。

    max_suction_force(float):单位吸取重量;
    
    partition:分区形式，行row和列col的个数;

    ToDo:
    自动分区也加入IO
    '''
    assert total_tip_dimension[0] >=  total_tip_dimension[1]
    t_l, t_w = total_tip_dimension
    all_tip_primitives = []
    all_tip_primitives_combinations = []
    for row in range(partition[0]): 
        for col in range(partition[1]):
            tip_primitive_id = "boxsuction_{}_{}".format(row,col)
            # tf_tip_region为当前吸取区域在Tip坐标系下的pose
            tf_tip_region = [0, 0, 0, 0, 0, 0, 1]
            tf_tip_region[0] = t_l / partition[1] * (col + 0.5 - partition[1] / 2)
            tf_tip_region[1] = t_w / partition[0] * (row + 0.5 - partition[0] / 2)
            tf_tip_region[2] = 0
            tip_primitiv_dimension = [t_l / partition[1], t_w / partition[0]]
            if t_l / partition[1] < t_w / partition[0]:
                tip_primitiv_dimension =  [t_w / partition[0], t_l / partition[1]]
                origin_pose = calculate_rotate_pose(Pose(*tf_tip_region), np.pi/2)
                tf_tip_region = pose_to_list(origin_pose)
            # 将tf_tip_region为转换为Tool坐标系下的pose
            tf_tool_region = transform_tip_to_tool(Pose(*tf_tip_region), tf_tool_tip)
            tip_primitive = bulid_tip_primitive(tip_primitive_id, pose_to_list(tf_tool_region), \
                                                tip_primitiv_dimension, max_suction_force, tp_io_config = None)
            all_tip_primitives.append(tip_primitive)

    for start_row, end_row in itertools.combinations(range(partition[0]+1), 2):
        for start_col, end_col in itertools.combinations(range(partition[1]+1), 2):
            single_tip_primitives_combination = []
            for row in range(start_row, end_row):
                for col in range(start_col, end_col):
                    single_tip_primitives_combination.append("boxsuction_{}_{}".format(row,col))
            all_tip_primitives_combinations.append(single_tip_primitives_combination)
    return all_tip_primitives, all_tip_primitives_combinations

def transform_tip_to_tool(origin_pose, relative_pose):
    '''
    将局部坐标系origin_pose转换到Tool原点坐标系下
    '''
    origin_pose_se3 = SE3(pose_to_list(origin_pose))
    relative_pose_se3 = SE3(relative_pose)
    target_pose = (relative_pose_se3 * origin_pose_se3).xyz_quat
    new_origin_pose = deepcopy(origin_pose)
    new_origin_pose.x = target_pose[0]
    new_origin_pose.y = target_pose[1]
    new_origin_pose.z = target_pose[2]
    new_origin_pose.qx = target_pose[3]
    new_origin_pose.qy = target_pose[4]
    new_origin_pose.qz = target_pose[5]
    new_origin_pose.qw = target_pose[6]
    return new_origin_pose

def calculate_rotate_pose(origin_pose, angle):
    '''
    计算origin_pose绕Z轴旋转一定角度之后的pose
    '''
    origin_pose_se3 = SE3(pose_to_list(origin_pose))
    Rz = tfm.rotation_matrix(angle, (0, 0, 1))
    rotate_pose = SE3(Rz)
    target_pose = (origin_pose_se3*rotate_pose).xyz_quat
    new_origin_pose = deepcopy(origin_pose)
    new_origin_pose.x = target_pose[0]
    new_origin_pose.y = target_pose[1]
    new_origin_pose.z = target_pose[2]
    new_origin_pose.qx = target_pose[3]
    new_origin_pose.qy = target_pose[4]
    new_origin_pose.qz = target_pose[5]
    new_origin_pose.qw = target_pose[6]
    return new_origin_pose

def build_tool(tool_name, defined_tip_primitives_combinations, defined_tip_primitives, \
                                                                    tip_shrink_height, defined_tool_collision,clamp_collision_list,clamp_collision_name):
    '''
    Inputs Data:

    tool_name(string):夹具名称

    defined_tip_primitives_combinations(nested list of tipprimitive.id):外部定义的最小吸取单位组合

    defined_tip_primitives(list of TipPrimitive):外部定义的最小吸取单位

    tip_shrink_height(float):吸盘吸取时的形变量

    defined_tool_collision(PrimitiveGroup):外部定义的吸盘碰撞体
    '''
    tool = Tool()
    tool.name = tool_name
    #将创建好的障碍物添加到tool上
    if len(defined_tool_collision.primitives) < 1:
        raise Exception("吸盘障碍物不能为空，吸盘障碍物至少包括吸盘上方的障碍物")
    tool.tool_collisions = defined_tool_collision
    tool.tool_collisions_list = clamp_collision_list
    tool.tool_collisions_name = clamp_collision_name
    #import ipdb;ipdb.set_trace()
    origin_all_tip_primitives = deepcopy(defined_tip_primitives)
    origin_tip_primimtives_dict = {tp.id : tp for tp in origin_all_tip_primitives}
    for i in range(len(defined_tip_primitives)): 
        for j in range(len(defined_tip_primitives[i].suction_areas)):
            if defined_tip_primitives[i].suction_areas[j].dimensions[0] < defined_tip_primitives[i].suction_areas[j].dimensions[1]:
                raise Exception(f"存在id为 {defined_tip_primitives[i].id} 的tip_primitive的长小于宽")
            tool.all_tip_primitives.append(defined_tip_primitives[i])

    def get_2d_corners(origin_pose, dimension):
        corners = []
        length, width = dimension
        center = np.array((origin_pose.x, origin_pose.y, origin_pose.z))
        origin_pose_matrix = SE3(pose_to_list(origin_pose)).homogeneous
        corner_helper = [[1,-1], [-1,-1], [-1,1], [1, 1]]
        for i in range(4):
            corner_point = [0.5 * length * corner_helper[i][0], 0.5 * width * corner_helper[i][1], center[2], 1]
            new_corner_point = np.matmul(origin_pose_matrix, corner_point)[0:2]
            corners.append(new_corner_point)
        return corners
    
    for i in range(len(defined_tip_primitives_combinations)):
        tip = Tip()
        tip.name = "tip_{}".format(i)
        single_tip_primitives_combination = defined_tip_primitives_combinations[i]
        tip.child_primitives_ids.extend(single_tip_primitives_combination)
        #如果tip只包含一个tip_primitive，需要保证tip和tip_primitive的Pose相同
        #如果tip包含多个tip_primitives，需要保证组合后tip的长一定沿tip_pose的X轴
        if len(single_tip_primitives_combination) == 1 and \
                len(origin_tip_primimtives_dict[single_tip_primitives_combination[0]].suction_areas) == 1:
            tip_primitive = origin_tip_primimtives_dict[single_tip_primitives_combination[0]]
            tip.tip_pose = tip_primitive.suction_areas[0].origin
            tip.bounding_box.type = RegionPrimitive.RECTANGLE
            tip.bounding_box.origin = tip.tip_pose
            tip.bounding_box.dimensions.extend(tip_primitive.suction_areas[0].dimensions) 
        else:
            points = []
            tip_z_list = []
            for tip_primitive_id in single_tip_primitives_combination:
                if tip_primitive_id not in list(origin_tip_primimtives_dict.keys()):
                    raise Exception(f"吸盘组合中id为{tip_primitive_id}的tip_primitive不在输入参数defined_tip_primitives中")
                tip_primitive = origin_tip_primimtives_dict[tip_primitive_id]
                for suction_region in tip_primitive.suction_areas:
                    points += get_2d_corners(suction_region.origin, suction_region.dimensions)
                    tip_z_list.append(suction_region.origin.z)
            if (max(tip_z_list) - min(tip_z_list)) > tip_shrink_height:
                raise Exception(f"defined_tip_primitives_combinations的第{i}个吸盘组合中的tip_primitives高度不一致")

            rect = sg.MultiPoint(np.array(points).astype(np.single)).minimum_rotated_rectangle
            coords = list(map(np.array, rect.exterior.coords[:]))
            center = (coords[2] + coords[0]) / 2
            edge1 = coords[1] - coords[0]
            edge2 = coords[3] - coords[0]
            if (np.linalg.norm(edge1) < np.linalg.norm(edge2)):
                edge1, edge2 = edge2, edge1
            length = np.linalg.norm(edge1)
            width = np.linalg.norm(edge2)
            dir1 = edge1 / length
            # Orthogonization
            edge2 = edge2 - edge1.dot(edge2) * edge2
            dir2 = edge2 / width
            dir2 /= np.cross(dir1, dir2)
            # back-project 2D center coordnates to big tip frame
            transform = tfm.identity_matrix()
            transform[0:2, 0] = dir1
            transform[0:2, 1] = dir2
            tip.tip_pose = Pose(*SE3(transform[0:3,0:3], [center[0], center[1], tip_z_list[0]]).xyz_quat)
            tip.bounding_box.type = RegionPrimitive.RECTANGLE
            tip.bounding_box.origin = tip.tip_pose
            tip.bounding_box.dimensions.extend([length, width]) 
        tip.soft_cup_height = 0.03
        tip.shrink_height = tip_shrink_height
        tool.tips.append(tip)
    #import ipdb;ipdb.set_trace()    
    return tool

def build_tool_old(length, width, height, sponge_height, quat, partition=(1,1)):
    length = float(length)
    width = float(width)
    height = float(height)
    sponge_height = float(sponge_height)

    tool = Tool()
    tool.name = "tool0"

    collision = PrimitiveGroup()
    collision.name = tool.name + "_collision"
    collision.origin.qx, collision.origin.qy, collision.origin.qz, collision.origin.qw = quat
    box_collision = GeometricPrimitive()
    box_collision.type = GeometricPrimitive.BOX
    box_collision.relative_pose.z = (height + sponge_height) / 2
    box_collision.relative_pose.qx, box_collision.relative_pose.qy, box_collision.relative_pose.qz, box_collision.relative_pose.qw = quat
    box_collision.dimensions.extend([length, width, height + sponge_height])
    collision.primitives.append(box_collision)
    tool.tool_collisions = collision

    for row in range(partition[0]):
        for col in range(partition[1]):
            tip_primitive = TipPrimitive()

            tip_suction_region = RegionPrimitive()
            tip_suction_region.type = RegionPrimitive.RECTANGLE
            tip_suction_region.origin.x = width / partition[0] * (row + 0.5 - partition[0] / 2)
            tip_suction_region.origin.y = length / partition[1] * (col + 0.5 - partition[1] / 2)
            tip_suction_region.origin.z = height
            tip_suction_region.origin.qx, tip_suction_region.origin.qy, tip_suction_region.origin.qz, tip_suction_region.origin.qw = quat
            tip_suction_region.dimensions.extend([length / partition[1], width / partition[0]])

            tip_primitive.id = "boxsuction_{}_{}".format(row,col)
            tip_primitive.suction_areas.append(tip_suction_region)
            tip_primitive.max_suction_force = 150.0
            tool.all_tip_primitives.append(tip_primitive)

    for start_row, end_row in itertools.combinations(range(partition[0]+1), 2):
        for start_col, end_col in itertools.combinations(range(partition[1]+1), 2):
            tip = Tip()
            tip.name = "tip_{}_{}_{}_{}".format(start_row, start_col, end_row - 1, end_col - 1)
            tip.tip_pose.x = (start_row + end_row - partition[0]) * width / partition[0] / 2
            tip.tip_pose.y = (start_col + end_col - partition[1]) * length / partition[1] / 2
            tip.tip_pose.z = height
            tip.tip_pose.qx, tip.tip_pose.qy, tip.tip_pose.qz, tip.tip_pose.qw = quat
            tip.bounding_box.type = RegionPrimitive.RECTANGLE
            tip.bounding_box.origin = tip.tip_pose
            tip.bounding_box.dimensions.extend([length / partition[0]*(end_row-start_row),
                                                width / partition[1]*(end_col-start_col)])
            tip.soft_cup_height = 0.03
            tip.shrink_height = 0.01
            for row in range(start_row, end_row):
                for col in range(start_col, end_col):
                    tip.child_primitives_ids.append("boxsuction_{}_{}".format(row,col))
            tool.tips.append(tip)
    return tool

def build_tool_less(length, width, height, sponge_height, relative_pose, partition=(1,1)):
    length = float(length)
    width = float(width)
    height = float(height)
    sponge_height = float(sponge_height)

    tool = Tool()
    tool.name = "tool0"
    quat = (relative_pose[3], relative_pose[4], relative_pose[5], relative_pose[6])
    collision = PrimitiveGroup()
    collision.name = tool.name + "_collision"
    collision.origin.qx, collision.origin.qy, collision.origin.qz, collision.origin.qw = quat
    box_collision = GeometricPrimitive()
    box_collision.type = GeometricPrimitive.BOX
    box_collision.relative_pose.x, box_collision.relative_pose.y, box_collision.relative_pose.z, box_collision.relative_pose.qx, box_collision.relative_pose.qy, box_collision.relative_pose.qz, box_collision.relative_pose.qw = relative_pose
    box_collision.dimensions.extend([length, width, height + sponge_height])
    collision.primitives.append(box_collision)
    tool.tool_collisions = collision

    for row in range(partition[0]):
        for col in range(partition[1]):
            tip_primitive = TipPrimitive()

            tip_suction_region = RegionPrimitive()
            tip_suction_region.type = RegionPrimitive.RECTANGLE
            tip_suction_region.origin.x = width / partition[0] * (row + 0.5 - partition[0] / 2)
            tip_suction_region.origin.y = length / partition[1] * (col + 0.5 - partition[1] / 2)
            tip_suction_region.origin.z = relative_pose[2] + height/2
            tip_suction_region.origin.qx, tip_suction_region.origin.qy, tip_suction_region.origin.qz, tip_suction_region.origin.qw = quat
            tip_suction_region.dimensions.extend([length / partition[1], width / partition[0]])

            tip_primitive.id = "boxsuction_{}_{}".format(row,col)
            tip_primitive.suction_areas.append(tip_suction_region)
            tip_primitive.max_suction_force = 150.0
            tool.all_tip_primitives.append(tip_primitive)
    
    start_row, end_row = 0, 1
    start_col = 0
    for end_col in range(1, partition[1] + 1):
        tip = Tip()
        tip.name = "tip_{}_{}_{}_{}".format(start_row, start_col, end_row - 1, end_col - 1)
        tip.tip_pose.x = (start_row + end_row - partition[0]) * width / partition[0] / 2
        tip.tip_pose.y = (start_col + end_col - partition[1]) * length / partition[1] / 2
        tip.tip_pose.z = relative_pose[2] + height/2
        tip.tip_pose.qx, tip.tip_pose.qy, tip.tip_pose.qz, tip.tip_pose.qw = quat
        tip.bounding_box.type = RegionPrimitive.RECTANGLE
        tip.bounding_box.origin = tip.tip_pose
        tip.bounding_box.dimensions.extend([length / partition[0]*(end_row-start_row),
                                            width / partition[1]*(end_col-start_col)])
        tip.soft_cup_height = 0.03
        tip.shrink_height = 0.01
        for row in range(start_row, end_row):
            for col in range(start_col, end_col):
                tip.child_primitives_ids.append("boxsuction_{}_{}".format(row,col))
        tool.tips.append(tip)


    return tool