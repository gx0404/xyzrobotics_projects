from xyz_env_manager.msg import Tool
from xyz_env_manager.msg import TipPrimitive
from xyz_env_manager.msg import RegionPrimitive
from xyz_env_manager.msg import Tip
from xyz_env_manager.msg import GeometricPrimitive
from xyz_env_manager.msg import PrimitiveGroup
from xyz_env_manager.msg import Pose
from xyz_env_manager.msg import BoxAlignOptions
from xyz_env_manager.msg import GraspPlanConfig

tool_1 = Tool()
tool_1.name = "tool1"
tip_1_collision_pose = Pose(0.29, 0, 0.10, 0, 0, 0.707106, 0.707106)
tool_1.tool_collisions.primitives.append(GeometricPrimitive(type=1, dimensions=[0.435, 0.28, 0.20], relative_pose=tip_1_collision_pose))
camera_collision_pose = Pose(0.32527896761894226, 0.5174153447151184, 0.08, -0.0007293656639392912, -0.0002543562670010954, 0.999991928934866, 0.003942764733489822)
tool_1.tool_collisions.primitives.append(GeometricPrimitive(type=1, dimensions=[0.391, 0.086, 0.09], relative_pose=camera_collision_pose))
board_collision_pose = Pose(0.4021695, 0.4021695, -0.14043035, 0.6115470541947413, 0.6399904100844013, 0.3211595256136631, 0.33656949636416134)
tool_1.tool_collisions.primitives.append(GeometricPrimitive(type=1, dimensions=[0.300, 0.250, 0.03], relative_pose=board_collision_pose))
tool_1.tool_collisions.color.r = 1
tool_1.tool_collisions.color.g = 0
tool_1.tool_collisions.color.b = 0

tool_1.tool_collisions.name = "box_tool"
tool_1.tool_collisions.origin.qw = 1
tipPrimitive1 = TipPrimitive()
tipPrimitive1.max_suction_force = 1.0
tipPrimitive1.id = "tip_primitive"
rp1 = RegionPrimitive()
rp1.type = rp1.RECTANGLE
rp1.dimensions = [0.435, 0.28]
rp1.origin = Pose(0.29, 0, 0.20, 0, 0, 0.707106, 0.707106)
tipPrimitive1.suction_areas.append(rp1)


tool_1.all_tip_primitives.append(tipPrimitive1)

tip = Tip()
tip.name = "total_tip"
tip.child_primitives_ids = ["tip_primitive"]
tip.tip_pose = Pose(0.29, 0, 0.17, 0, 0, 0.707106, 0.707106)

tool_1.tips.append(tip)


box_l = 0.4
box_w = 0.3
box_l_w_half = (box_l + box_w) * 0.5
box_l_half = box_l * 0.5
box_h = 0.25
num_vertices_test = 8
poses = [[-box_l_w_half, -box_w, box_h, 0, 0, 0, 1],
        [-box_l_w_half, 0, box_h, 0, 0, 0, 1],
        [-box_l_w_half, box_w, box_h, 0, 0, 0, 1],
        [0, box_l_half, box_h, 0, 0, 0.70710678, 0.70710678],
        [0, -box_l_half, box_h, 0, 0, 0.70710678, 0.70710678],
        [box_l_w_half, -box_w, box_h, 0, 0, 0, 1],
        [box_l_w_half, 0, box_h, 0, 0, 0, 1],
        [box_l_w_half, box_w, box_h, 0, 0, 0, 1]]

primitive_groups = []

for index, pose in enumerate(poses):
    pg = PrimitiveGroup()
    pg.origin.x = pose[0]
    pg.origin.y = pose[1]
    pg.origin.z = pose[2]
    pg.origin.qx = pose[3]
    pg.origin.qy = pose[4]
    pg.origin.qz = pose[5]
    pg.origin.qw = pose[6]
    pg.name = str(index)
    gp = GeometricPrimitive()
    gp.type = gp.BOX
    gp.dimensions = [0, 0, 0]
    gp.dimensions[0] = box_l
    gp.dimensions[1] = box_w
    gp.dimensions[2] = box_h
    gp.relative_pose.z = -box_h/2.0
    gp.relative_pose.qw = 1

    pg.primitives.append(gp)
    primitive_groups.append(pg)

from xyz_motion import GraspPlanGenerator

gpg = GraspPlanGenerator.from_ros_msg(tool_1, primitive_groups)
bao= BoxAlignOptions()
bao.th_z_diff = 0.04
bao.th_angle_diff = 5
bao.th_rel_center_align_diff = 0.06
bao.th_rel_penetration = 0.08
bao.th_rel_max_gap = 0.5
bao.flag_180_symmetry = True
bao.flag_allow_length_align = True
bao.flag_allow_width_align = False

grasp_plan_config = GraspPlanConfig()
grasp_plan_config.max_suction_ratio_noncontact =  0.025
grasp_plan_config.min_suction_ratio_contact = 0.5
grasp_plan_config.top_k = 100
grasp_plan_config.pallet_algo_type = grasp_plan_config.HOMOGPALLET
kk = gpg.generate_grasp_plan(bao, grasp_plan_config)

import ipdb; ipdb.set_trace()