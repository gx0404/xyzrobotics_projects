import numpy as np
import rospkg
from xyz_env_manager.msg import Pallet
from xyz_env_manager.msg import Conveyor
from xyz_env_manager.msg import Workspace
from xyz_env_manager.msg import Pose

from xyz_env_manager.msg import RobotConfig
from xyz_env_manager.msg import JointLimits
from xyz_env_manager.msg import PrimitiveGroup
from xyz_env_manager.msg import GeometricPrimitive

def build_workspace(workspace_id, bottom_pose, dimensions, ws_type, ignore_collision = False):
    check_dimensions_schema(dimensions, ws_type)
    workspace = Workspace()
    workspace.workspace_id = workspace_id
    workspace.bottom_pose = Pose(bottom_pose[0], bottom_pose[1], bottom_pose[2], bottom_pose[3], bottom_pose[4], bottom_pose[5], bottom_pose[6])
    workspace.dimensions = dimensions
    if ws_type == "conveyor":
        workspace.type = workspace.CONVEYOR
    elif ws_type == "pallet":
        workspace.type = workspace.PALLET
    else:
        raise Exception("工作空间类型应在pallet和conveyor中选择一种!")
    workspace.ignore_collision = ignore_collision
    return workspace

def build_pallet(name, top_origin, dimensions):
    check_dimensions_schema(dimensions)
    pallet = Pallet()
    pallet.name = name
    pallet.length = dimensions[0]
    pallet.width = dimensions[1]
    pallet.height = dimensions[2]
    pallet.top_origin = Pose(top_origin[0], top_origin[1], top_origin[2], top_origin[3], top_origin[4], top_origin[5], top_origin[6])
    return pallet

def build_conveyor(name, top_origin, dimensions, side_height, side_width):
    check_dimensions_schema(dimensions)
    conveyor = Conveyor()
    conveyor.name = name
    conveyor.length = dimensions[0]
    conveyor.width = dimensions[1]
    conveyor.height = dimensions[2]
    conveyor.side_height = side_height
    conveyor.side_width = side_width
    conveyor.top_origin = Pose(top_origin[0], top_origin[1], top_origin[2], top_origin[3], top_origin[4], top_origin[5], top_origin[6])
    return conveyor

def build_collision(name, origin, dimensions, geometric_type, alpha):
    check_dimensions_schema(dimensions, geometric_type)
    collision = PrimitiveGroup(name=name, origin=Pose(origin[0], origin[1], origin[2], origin[3], origin[4], origin[5], origin[6]))
    collision.color.a = alpha
    collision.primitives.append(GeometricPrimitive(type=geometric_type, dimensions=dimensions, relative_pose=Pose(0, 0, 0, 0, 0, 0, 1)))
    return collision  

def write_system_config(robot_id, robot_description_name, child_model):
    import xml.etree.ElementTree as ET
    tree = ET.parse("../launch/system_config.launch")
    root = tree.getroot()
    for child in root:
        if child.tag == "include":
            splitted_str = child.attrib["file"].split("/")
            if child_model != "":
                splitted_str[2] = "load_" + robot_description_name.split("_")[1] + "_" + child_model + ".launch"
            else:
                splitted_str[2] = "load_" + robot_description_name.split("_")[1] + ".launch"
            splitted_str[0] = '$(find {}_support)'.format(robot_description_name)
            child.attrib["file"] = "/".join(splitted_str)            
            tree.write('../launch/system_config.launch')
            break

def set_robot_config(robot_id, robot_description_name, child_model, base_transform, upper_limit, lower_limit):
    rospack = rospkg.RosPack()
    write_system_config(robot_id, robot_description_name, child_model)
    
    robot_config = RobotConfig()
    robot_config.robot_id = robot_id
    robot_config.robot_description_name = robot_description_name
    robot_config.child_model = child_model
    robot_config.base_transform = Pose(*base_transform)
    limits = []
    upper_limit = [np.radians(ul) for ul in upper_limit]
    lower_limit = [np.radians(ll) for ll in lower_limit]
    for _idx, up_limit in enumerate(upper_limit):
        low_limit = lower_limit[_idx]
        jm = JointLimits()
        jm.joint_index = _idx
        jm.max_position = up_limit
        jm.min_position = low_limit
        robot_config.joint_limits.append(jm)
    return robot_config

def check_dimensions_schema(dimensions, dim_source = "default"):
    if not isinstance(dimensions,list):
        raise Exception("dimensions的数据类型必须是一个列表！")
    
    if dim_source == "conveyor" or dim_source == "pallet":
        pass
    else:
        if dim_source == GeometricPrimitive.CYLINDER and len(dimensions) != 2:
            raise Exception("圆柱体的尺寸只能包含两个参数！")
        elif dim_source != GeometricPrimitive.CYLINDER and len(dimensions) != 3:
            raise Exception("除圆柱之外的碰撞体尺寸只能包含三个参数！")
        else:
            pass
        if not all(dim > 0.1 for dim in dimensions):
            raise Exception("碰撞体的任意方向尺寸必须大于0.1米！")
    