import numpy as np
from xyz_env_manager.msg import Pallet
from xyz_env_manager.msg import Conveyor
from xyz_env_manager.msg import Workspace
from xyz_env_manager.msg import PlanningEnvironment
from xyz_env_manager.msg import Pose
from xyz_motion import PlanningEnvironmentRos
from xyz_motion import WorkspaceRos
from xyz_env_manager.msg import  AllRobotToolStates
from xyz_env_manager.msg import RobotToolState
from xyz_env_manager.msg import RobotConfig
from xyz_env_manager.msg import JointLimits
from xyz_env_manager.msg import Tool
from xyz_env_manager.msg import TipPrimitive
from xyz_env_manager.msg import RegionPrimitive
from xyz_env_manager.msg import Tip
from xyz_env_manager.msg import GeometricPrimitive
from xyz_env_manager.msg import PrimitiveGroup
from xyz_env_manager.msg import JointLimits, IO
from xyz_motion import AllRobotToolStatesRos
from tool_builder import build_tool_less
from tool_builder import build_tool, cal_tip_primitives_combination, bulid_tip_primitive
from workspace_builder import build_workspace, build_pallet, build_conveyor, build_collision, set_robot_config

#机器人底座1.605m
# 拣配来料空间0-码垛托盘0
ws0 = build_workspace(workspace_id="0", bottom_pose=[-0.15, 1.85+0.02, -1.121, 0.0, 0.0, 0.0, 1], dimensions=[1.25, 1.05+0.2, 1.65], ws_type="pallet")
pallet_0 = build_pallet(name="pallet_0", top_origin=[-0.15, 1.85+0.02,-1.121-0.04, 0.0, 0.0, 0.0, 1], dimensions=[1.25, 1.05+0.2, 0.15+0.334])
ws0.pallet = pallet_0


# 缓存区1-码垛托盘1
ws1 = build_workspace(workspace_id="1", bottom_pose=[-2, 1.1, -1.121, 0.0, 0.0, 0.707, 0.707], dimensions=[1.25, 1.05, 1.65], ws_type="pallet")
pallet_1 = build_pallet(name="pallet_1", top_origin=[-2, 1.1, -1.121-0.04, 0.0, 0.0, 0.707, 0.707], dimensions=[1.2, 1.0, 0.15+0.334])
ws1.pallet = pallet_1

#笼车空间2-码垛托盘2
ws2 = build_workspace(workspace_id="2", bottom_pose=[1.6+0.1, 1.65, -1.267, 0.0, 0.0, 0.707, 0.707], dimensions=[1.22, 0.815, 1.62], ws_type="pallet")
pallet_2 = build_pallet(name="pallet_2", top_origin=[1.6+0.1, 1.65, -1.267-0.04, 0.0, 0.0, 0.707, 0.707], dimensions=[1.22, 0.815, 0.33])
ws2.pallet = pallet_2

#笼车空间3-码垛托盘3
ws3 = build_workspace(workspace_id="3", bottom_pose=[2, -0.3, -1.267, 0.0, 0.0, 0.707, 0.707], dimensions=[1.22, 0.815, 1.62], ws_type="pallet")
pallet_3 = build_pallet(name="pallet_3", top_origin=[2, -0.3, -1.267-0.04, 0.0, 0.0, 0.707, 0.707], dimensions=[1.22, 0.815, 0.33])
ws3.pallet = pallet_3

#码垛托盘4
ws4 = build_workspace(workspace_id="4", bottom_pose=[-1.92, -0.9, -1.121, 0.0, 0.0, 0.707, 0.707], dimensions=[1.25, 1.05, 1.65], ws_type="pallet")
pallet_4 = build_pallet(name="pallet_4", top_origin=[-1.92, -0.9, -1.121-0.04, 0.0, 0.0, 0.707, 0.707], dimensions=[1.2, 1.0, 0.15+0.334])
ws4.pallet = pallet_4

#码垛托盘5
ws5 = build_workspace(workspace_id="5", bottom_pose=[-0.5, -2.4, -1.121, 0.0, 0.0, 0.0, 1], dimensions=[1.25, 1.05, 1.65], ws_type="pallet")
pallet_5 = build_pallet(name="pallet_5", top_origin=[-0.5, -2.4, -1.121-0.04, 0.0, 0.0, 0.0, 1], dimensions=[1.2, 1.0, 0.15+0.334])
ws5.pallet = pallet_5

#输送线6
ws6 = build_workspace(workspace_id="6", bottom_pose=[1.139, -1.818+0.035, -1-0.005, 0.0, 0.0, 0.707, 0.707], dimensions=[0.61, 0.41, 0.6], ws_type="conveyor")
conveyor_6 = build_conveyor(name="conveyor_6", top_origin=[1.139, -2.221, -1-0.005, 0.0, 0.0, 0.707, 0.707], dimensions=[2, 0.45, 0.5], side_height=0.12, side_width=0.01)
ws6.conveyor = conveyor_6


#混码缓存位7
ws7 = build_workspace(workspace_id="7", bottom_pose=[1.298-0.41-0.07, -2.645, -0.456-0.02, 0.0, 0.0, 0, 1], dimensions=[0.4, 0.3, 0.4], ws_type="pallet")
pallet_7 = build_pallet(name="pallet_7", top_origin=[1.298-0.41-0.07, -2.645, -0.456-0.02, 0.0, 0.0, 0, 1], dimensions=[1, 0.3, 0.11])
ws7.pallet = pallet_7

#混码缓存位8
ws8 = build_workspace(workspace_id="8", bottom_pose=[1.298, -2.638, -0.456-0.02, 0.0, 0.0, 0, 1], dimensions=[0.4, 0.3, 0.4], ws_type="pallet")
pallet_8 = build_pallet(name="pallet_8", top_origin=[1.298, -2.638, -0.456-0.02, 0.0, 0.0, 0, 1], dimensions=[1, 0.3, 0.11])
ws8.pallet = pallet_8


# 创建简单障碍物(安全网、相机支架、控制柜、机器人自碰撞等)

#笼车工作空间障碍物
collision_pallet_2 = build_collision(name="collision_pallet_2", origin=[1.6+0.1,1.65+1.22/2+0.11/2, -1.267+1.7/2, 0, 0, 0, 1], dimensions=[0.87+0.1, 0.11,1.8], geometric_type=GeometricPrimitive.BOX, alpha=0.5)

#笼车工作空间障碍物
collision_pallet_3 = build_collision(name="collision_pallet_3", origin=[2, -0.3+1.22/2+0.11/2, -1.267+1.7/2, 0, 0, 0, 1], dimensions=[0.87+0.1, 0.11,1.8], geometric_type=GeometricPrimitive.BOX, alpha=0.5)


#前围栏
collision_1 = build_collision(name="col_1", origin=[2.705+0.09, 0, -0.32+1.5/2, 0, 0, 0, 1], dimensions=[0.11, 6.59,2.3+1.5], geometric_type=GeometricPrimitive.BOX, alpha=0.2)

# #后围栏
# collision_2 = build_collision(name="col_2", origin=[-2.82+0.12, 0-5.25/2, -0.32+1.5/2, 0, 0, 0, 1], dimensions=[0.11, 6.59-5.25,2.3+1.5], geometric_type=GeometricPrimitive.BOX, alpha=0.2)
#后围栏
collision_15 = build_collision(name="col_15", origin=[-2.85-0.032, 0, -0.32+1.5/2, 0, 0, 0, 1], dimensions=[0.11, 6,5.3+1.5], geometric_type=GeometricPrimitive.BOX, alpha=0.2)
# #后围栏
# collision_16 = build_collision(name="col_16", origin=[-2.82+0.12, 0+5.25/2, -0.32+1.5/2, 0, 0, 0, 1], dimensions=[0.11, 6.59-5.25,2.3+1.5], geometric_type=GeometricPrimitive.BOX, alpha=0.2)

#左围栏
collision_3 = build_collision(name="col_3", origin=[0.052, 3.098+0.11/2, -0.32+1.5/2, 0, 0, 0, 1], dimensions=[5.355, 0.14,2.3+1.5], geometric_type=GeometricPrimitive.BOX, alpha=0.2)

#右围栏
collision_4 = build_collision(name="col_4", origin=[0.052, -3.5, -0.32+1.5/2, 0, 0, 0, 1], dimensions=[5.355, 0.11,2.3+1.5], geometric_type=GeometricPrimitive.BOX, alpha=0.2)

#屋顶
collision_13 = build_collision(name="col_13", origin=[0, 0 ,3, 0, 0, 0, 1], dimensions=[5, 5,0.11], geometric_type=GeometricPrimitive.BOX, alpha=0.1)

#相机立柱
collision_5 = build_collision(name="col_5", origin=[0.9430, 2.824, -0.07000, 0, 0, 0, 1], dimensions=[2.8, 0.2+0.02], geometric_type=GeometricPrimitive.CYLINDER, alpha=0.2)
#相机立柱2
collision_18 = build_collision(name="col_18", origin=[0.9430, 2.824, 1.5, 0, 0, 0, 1], dimensions=[2, 0.17], geometric_type=GeometricPrimitive.CYLINDER, alpha=0.2)

#相机
collision_6 = build_collision(name="col_6", origin=[-0.04, 1.9, 2.43+0.04, 0, 0, 0, 1], dimensions=[1.5,1.5,0.25], geometric_type=GeometricPrimitive.BOX, alpha=0.2)

#相机斜杆
collision_7 = build_collision(name="col_7", origin=[0.05, 2.4, 2.3,  0.08682408, -0.49240388,  0.15038371,  0.85286854], dimensions=[1.5, 0.11], geometric_type=GeometricPrimitive.CYLINDER, alpha=0.2)

#电气柜
collision_8 = build_collision(name="col_8", origin=[2.05, -2.654, -0.47+1/2, 0, 0, 0.707, 0.707], dimensions=[1.54, 0.71,2+1], geometric_type=GeometricPrimitive.BOX, alpha=0.2)

#机器人底座
collision_9 = build_collision(name="col_9", origin=[0, 0, -0.8, 0, 0, 0, 1], dimensions=[1.25, 1.25,1.47], geometric_type=GeometricPrimitive.BOX, alpha=0.2)

#输送线扫码器1
collision_10 = build_collision(name="col_10", origin=[1, -1.0, -0.456-1.1/2, 0, 0, 0, 1], dimensions=[0.8, 0.11,1.2], geometric_type=GeometricPrimitive.BOX, alpha=0.2)

#输送线扫码器2
collision_11 = build_collision(name="col_11", origin=[1.67, -2, -0.456-1.1/2, 0, 0, 0.707, 0.707], dimensions=[0.6, 0.22,1.3], geometric_type=GeometricPrimitive.BOX, alpha=0.2)

#输送线缓存支架
collision_12 = build_collision(name="col_12", origin=[1.15, -2.57, -0.456-1.1/2-0.1, 0, 0, 0, 1], dimensions=[0.430, 0.400,1.1], geometric_type=GeometricPrimitive.BOX, alpha=0.2)

#机器人航空插头
collision_14 = build_collision(name="col_14", origin=[-0.8, -0.15, -0.2, 0, 0, 0, 1], dimensions=[0.15, 0.7,0.94], geometric_type=GeometricPrimitive.BOX, alpha=0.2)

#相机立柱扫码器
collision_17 = build_collision(name="col_17", origin=[1.07, 2.75, 0.73, 0.0, 0.0, 0.08715573006937669, 0.996194699207021], dimensions=[0.11, 0.35,0.33], geometric_type=GeometricPrimitive.BOX, alpha=0.2)


# 创建复杂障碍物(由多个几何体组成，一般不需要)，可以参考以下代码
# combined_collision = PrimitiveGroup(name="col_combined", origin=Pose(2.7041, -0.802, 0.8, 0, 0, 0, 1))
# combined_collision.color.a = 0.8
# combined_collision.color.r = 1
# combined_collision.color.g = 1
# combined_collision.color.b = 1
# combined_collision.primitives.append(GeometricPrimitive(type=GeometricPrimitive.CYLINDER, dimensions=[3.0, 0.1], relative_pose=Pose(0, 0, 0, 0, 0, 0, 1)))
# combined_collision.primitives.append(GeometricPrimitive(type=GeometricPrimitive.BOX, dimensions=[1.0, 1.0, 0.1], relative_pose=Pose(0, 0, 0, 0, 0, 0, 1)))

# 定义整个工作环境
pe = PlanningEnvironment()
pe.name = "workspace_env"

# 将之前创建的工作空间和障碍物加入工作环境
pe.workspaces.append(ws0)
pe.workspaces.append(ws1)
pe.workspaces.append(ws2)
pe.workspaces.append(ws3)
pe.workspaces.append(ws4)
pe.workspaces.append(ws5)
pe.workspaces.append(ws6)
pe.workspaces.append(ws7)
pe.workspaces.append(ws8)

pe.collision_objects.append(collision_1)
# pe.collision_objects.append(collision_2)
pe.collision_objects.append(collision_3)
pe.collision_objects.append(collision_4)
pe.collision_objects.append(collision_5)
pe.collision_objects.append(collision_6)
#pe.collision_objects.append(collision_7)
pe.collision_objects.append(collision_8)
pe.collision_objects.append(collision_9)
pe.collision_objects.append(collision_10)
pe.collision_objects.append(collision_11)
pe.collision_objects.append(collision_12)
pe.collision_objects.append(collision_13)
pe.collision_objects.append(collision_14)
pe.collision_objects.append(collision_15)
pe.collision_objects.append(collision_17)
pe.collision_objects.append(collision_18)
# pe.collision_objects.append(collision_pallet_2)
# pe.collision_objects.append(collision_pallet_3)
# pe.collision_objects.append(combined_collision)

workspace_env = PlanningEnvironmentRos.from_ros_msg(pe)



# 设定机器人配置
## robot_description_name指的是安装xyz-urdf后的机器相关型号的描述名称， 
## 例如这里可以使用 roscd fanuc_r2000ic_support 进入目录
all_robots = AllRobotToolStates()
robot_config = set_robot_config(robot_id="0", robot_description_name="kuka_kr180", child_model="r3200_pa", \
                                base_transform=[0, 0, 0, 0, 0, 0, 1], upper_limit=[185, -5.05, 154.5, 350], lower_limit=[-185, -139.95, 0, -350])
all_robots.robot_configs.append(robot_config)



#创建夹具 
# 400*300*230 中欧箱

tool_name = "tool1"

tip_primitive_1 = bulid_tip_primitive(tool_name = tool_name, id="tp_1", tf_tool_region=([0.128433+0.002,
 0.0,
 0.782879,
 0.0,
 0.0,
 -0.7071,
 0.7071]), dimension=[0.4, 0.3], max_suction_force=150)

all_tip_primitives = [tip_primitive_1]

combined_tool_collision = PrimitiveGroup(name="tool_collision", origin=Pose(0, 0, 0, 0, 0, 0, 1))



clamp_collision_list = []
clamp_collision_name = []

#相机
combined_tool_collision.primitives.append(GeometricPrimitive(type=GeometricPrimitive.BOX, dimensions=[0.55, 0.3, 0.23], relative_pose=Pose(-0.352-0.209/2, 0, 0.0835+0.03/2, 0, 0, 0.707, 0.707)))

clamp_collision_camera = PrimitiveGroup(name="camera", origin=Pose(0, 0, 0, 0, 0, 0, 1))
clamp_collision_camera.primitives.append(GeometricPrimitive(type=GeometricPrimitive.BOX, dimensions=[0.55, 0.3+0.03, 0.25], relative_pose=Pose(-0.352-0.209/2, 0, 0.0835+0.05/2, 0, 0, 0.707, 0.707)))
clamp_collision_list.append(clamp_collision_camera)
clamp_collision_name.append("camera")

#夹具大本体
combined_tool_collision.primitives.append(GeometricPrimitive(type=GeometricPrimitive.BOX, dimensions=[0.700, 0.552, 0.7], relative_pose=Pose(0, 0, 0.08+0.7/2, 0, 0, 0, 1)))
# combined_tool_collision.primitives.append(GeometricPrimitive(type=GeometricPrimitive.BOX, dimensions=[0.4-0.015, 0.6-0.015, 0.7], relative_pose=Pose(0.12843-0.3/2,
#  0,
#  0.1+0.7/2-0.03,
#  0.0,
#  0.0,
#  -0.7071,
#  0.7071)))

clamp_collision_body = PrimitiveGroup(name="body", origin=Pose(0, 0, 0, 0, 0, 0, 1))
clamp_collision_body.primitives.append(GeometricPrimitive(type=GeometricPrimitive.BOX, dimensions=[0.4-0.015, 0.6-0.015, 0.65], relative_pose=Pose(0.12843-0.3/2,
 0,
 0.1+0.65/2-0.03,
 0.0,
 0.0,
 -0.7071,
 0.7071)))
clamp_collision_list.append(clamp_collision_body)
clamp_collision_name.append("body")

 #夹具大本体上半部分
combined_tool_collision.primitives.append(GeometricPrimitive(type=GeometricPrimitive.BOX, dimensions=[0.4-0.015, 0.7, 0.3+0.2], relative_pose=Pose(0.12843-0.5/2,
 0,
 0.1+0.3/2+0.2/2,
 0.0,
 0.0,
 -0.7071,
 0.7071)))
clamp_collision_body = PrimitiveGroup(name="body_1", origin=Pose(0, 0, 0, 0, 0, 0, 1))
clamp_collision_body.primitives.append(GeometricPrimitive(type=GeometricPrimitive.BOX, dimensions=[0.4-0.015, 0.7, 0.3], relative_pose=Pose(0.12843-0.5/2,
 0,
 0.08+0.3/2+0.2/2,
 0.0,
 0.0,
 -0.7071,
 0.7071)))
clamp_collision_list.append(clamp_collision_body)
clamp_collision_name.append("body_1")

#气缸挡板
#夹具短边x方向挡板
clamp_collision_0 = PrimitiveGroup(name="x", origin=Pose(0, 0, 0, 0, 0, 0, 1))
clamp_collision_0.primitives.append(GeometricPrimitive(type=GeometricPrimitive.BOX, dimensions=[0.11, 0.34, 0.15+0.3], relative_pose=Pose(-0.07-0.05, 0, 0.660+0.1-0.3/2, 0, 0, 0, 1)))
clamp_collision_list.append(clamp_collision_0)
clamp_collision_name.append("x")

combined_tool_collision.primitives.append(GeometricPrimitive(type=GeometricPrimitive.BOX, dimensions=[0.11, 0.34, 0.25+0.3], relative_pose=Pose(-0.07-0.05, 0, 0.660+0.1-0.3/2, 0, 0, 0, 1)))

#夹具长边+y方向挡板
clamp_collision_1 = PrimitiveGroup(name="+y", origin=Pose(0, 0, 0, 0, 0, 0, 1))
clamp_collision_1.primitives.append(GeometricPrimitive(type=GeometricPrimitive.BOX, dimensions=[0.24+0.13, 0.11, 0.15+0.3], relative_pose=Pose(0.157-0.008-0.13/2, 0.249+0.05, 0.660+0.1-0.3/2, 0, 0, 0, 1)))
clamp_collision_list.append(clamp_collision_1)
clamp_collision_name.append("+y")

combined_tool_collision.primitives.append(GeometricPrimitive(type=GeometricPrimitive.BOX, dimensions=[0.24+0.13, 0.11, 0.25+0.3], relative_pose=Pose(0.157-0.008-0.13/2, 0.249+0.05, 0.660+0.1-0.3/2, 0, 0, 0, 1)))

#夹具长边-y方向挡板
clamp_collision_1 = PrimitiveGroup(name="-y", origin=Pose(0, 0, 0, 0, 0, 0, 1))
clamp_collision_1.primitives.append(GeometricPrimitive(type=GeometricPrimitive.BOX, dimensions=[0.24+0.13, 0.11, 0.15+0.3], relative_pose=Pose(0.157-0.008-0.13/2, -0.249-0.05, 0.660+0.1-0.3/2, 0, 0, 0, 1)))
clamp_collision_list.append(clamp_collision_1)
clamp_collision_name.append("-y")

combined_tool_collision.primitives.append(GeometricPrimitive(type=GeometricPrimitive.BOX, dimensions=[0.24+0.13, 0.11, 0.25+0.3], relative_pose=Pose(0.157-0.008-0.13/2, -0.249-0.05, 0.660+0.1-0.3/2, 0, 0, 0, 1)))


all_tip_primitives_combinations = [["tp_1"]]

tool_1 = build_tool(tool_name = tool_name, defined_tip_primitives_combinations = all_tip_primitives_combinations, \
                                                            defined_tip_primitives = all_tip_primitives, tip_shrink_height = 0.015, \
                                                            defined_tool_collision = combined_tool_collision,clamp_collision_list=clamp_collision_list,clamp_collision_name=clamp_collision_name)

tool_1.mounting_transform = Pose(0, 0, 0, 0, 0, 0, 1)


#创建夹具 
# 600*400*230 

tool_name = "tool2"

# tip_primitive_1 = bulid_tip_primitive(tool_name = tool_name, id="tp_1", tf_tool_region=([0.12964,
#  0.00054,
#  0.7646,
#  0.0,
#  -0.0,
#  -0.0,
#  1]), dimension=[0.6, 0.4], max_suction_force=150)

tip_primitive_1 = bulid_tip_primitive(tool_name = tool_name, id="tp_1", tf_tool_region=([0.13064+0.00075,
 -0.00046,
 0.7646,
 0.0,
 -0.0,
 -0.0,
 1]), dimension=[0.6, 0.4], max_suction_force=150)

all_tip_primitives = [tip_primitive_1]

combined_tool_collision = PrimitiveGroup(name="tool_collision", origin=Pose(0, 0, 0, 0, 0, 0, 1))

clamp_collision_list = []
clamp_collision_name = []
#相机
combined_tool_collision.primitives.append(GeometricPrimitive(type=GeometricPrimitive.BOX, dimensions=[0.55, 0.3, 0.23], relative_pose=Pose(-0.352-0.209/2, 0, 0.0835+0.03/2, 0, 0, 0.707, 0.707)))

clamp_collision_camera = PrimitiveGroup(name="camera", origin=Pose(0, 0, 0, 0, 0, 0, 1))
clamp_collision_camera.primitives.append(GeometricPrimitive(type=GeometricPrimitive.BOX, dimensions=[0.55, 0.3+0.03, 0.25], relative_pose=Pose(-0.352-0.209/2, 0, 0.0835+0.05/2, 0, 0, 0.707, 0.707)))
clamp_collision_list.append(clamp_collision_camera)
clamp_collision_name.append("camera")

#夹具大本体
combined_tool_collision.primitives.append(GeometricPrimitive(type=GeometricPrimitive.BOX, dimensions=[0.700, 0.552, 0.7], relative_pose=Pose(0, 0, 0.08+0.7/2, 0, 0, 0, 1)))


clamp_collision_body = PrimitiveGroup(name="body", origin=Pose(0, 0, 0, 0, 0, 0, 1))
clamp_collision_body.primitives.append(GeometricPrimitive(type=GeometricPrimitive.BOX, dimensions=[0.4-0.015, 0.6-0.015, 0.65], relative_pose=Pose(0.12843-0.3/2,
 -0.0009011965213759901,
 0.1+0.65/2-0.03,
 0.0,
 0.0,
 -0.7071,
 0.7071)))
clamp_collision_list.append(clamp_collision_body)
clamp_collision_name.append("body")




 #夹具大本体上半部分
combined_tool_collision.primitives.append(GeometricPrimitive(type=GeometricPrimitive.BOX, dimensions=[0.4-0.015, 0.7, 0.3+0.2], relative_pose=Pose(0.12843-0.5/2,
 0,
 0.1+0.3/2+0.2/2,
 0.0,
 0.0,
 -0.7071,
 0.7071)))
clamp_collision_body = PrimitiveGroup(name="body_1", origin=Pose(0, 0, 0, 0, 0, 0, 1))
clamp_collision_body.primitives.append(GeometricPrimitive(type=GeometricPrimitive.BOX, dimensions=[0.4-0.015, 0.7, 0.3], relative_pose=Pose(0.12843-0.5/2,
 0,
 0.08+0.3/2+0.2/2,
 0.0,
 0.0,
 -0.7071,
 0.7071)))
clamp_collision_list.append(clamp_collision_body)
clamp_collision_name.append("body_1")

#气缸挡板
#夹具短边x方向挡板
clamp_collision_0 = PrimitiveGroup(name="x", origin=Pose(0, 0, 0, 0, 0, 0, 1))
clamp_collision_0.primitives.append(GeometricPrimitive(type=GeometricPrimitive.BOX, dimensions=[0.11, 0.34, 0.15+0.3], relative_pose=Pose(-0.219-0.05, 0, 0.660+0.1-0.3/2, 0, 0, 0, 1)))
clamp_collision_list.append(clamp_collision_0)
clamp_collision_name.append("x")

combined_tool_collision.primitives.append(GeometricPrimitive(type=GeometricPrimitive.BOX, dimensions=[0.11, 0.34, 0.25+0.3], relative_pose=Pose(-0.219-0.05, 0, 0.660+0.1-0.3/2, 0, 0, 0, 1)))

#夹具长边+y方向挡板
clamp_collision_1 = PrimitiveGroup(name="+y", origin=Pose(0, 0, 0, 0, 0, 0, 1))
clamp_collision_1.primitives.append(GeometricPrimitive(type=GeometricPrimitive.BOX, dimensions=[0.24+0.13, 0.11, 0.15+0.3], relative_pose=Pose(0.157-0.008-0.13/2, 0.249+0.05, 0.660+0.1-0.3/2, 0, 0, 0, 1)))
clamp_collision_list.append(clamp_collision_1)
clamp_collision_name.append("+y")

combined_tool_collision.primitives.append(GeometricPrimitive(type=GeometricPrimitive.BOX, dimensions=[0.24+0.13, 0.11, 0.25+0.3], relative_pose=Pose(0.157-0.008-0.13/2, 0.249+0.05, 0.660+0.1-0.3/2, 0, 0, 0, 1)))

#夹具长边-y方向挡板
clamp_collision_1 = PrimitiveGroup(name="-y", origin=Pose(0, 0, 0, 0, 0, 0, 1))
clamp_collision_1.primitives.append(GeometricPrimitive(type=GeometricPrimitive.BOX, dimensions=[0.24+0.13, 0.11, 0.15+0.3], relative_pose=Pose(0.157-0.008-0.13/2, -0.249-0.05, 0.660+0.1-0.3/2, 0, 0, 0, 1)))
clamp_collision_list.append(clamp_collision_1)
clamp_collision_name.append("-y")

combined_tool_collision.primitives.append(GeometricPrimitive(type=GeometricPrimitive.BOX, dimensions=[0.24+0.13, 0.11, 0.25+0.3], relative_pose=Pose(0.157-0.008-0.13/2, -0.249-0.05, 0.660+0.1-0.3/2, 0, 0, 0, 1)))



all_tip_primitives_combinations = [["tp_1"]]

tool_2 = build_tool(tool_name = tool_name, defined_tip_primitives_combinations = all_tip_primitives_combinations, \
                                                            defined_tip_primitives = all_tip_primitives, tip_shrink_height = 0.015, \
                                                            defined_tool_collision = combined_tool_collision,clamp_collision_list=clamp_collision_list,clamp_collision_name=clamp_collision_name)

tool_2.mounting_transform = Pose(0, 0, 0, 0, 0, 0, 1)





# # 定义吸盘/夹具
# # 一共有6步：
# ## 第 1 步:声明 tool 的名字
# tool_name = 'tool0'

# ## 第 2 步:自动计算吸盘分区的所有tip_primitives及能够形成吸盘的tip_primitives组合
# ##         需要注意的是，自动计算吸盘分区需要满足一定条件，可以阅读相关confluence文档
# ##         如果满足条件，完成第1步后可直接到第3步
# ##         如果不满足条件请将第一步代码注释，跳过第1步，需要完成第2步
# all_tip_primitives, all_tip_primitives_combinations = cal_tip_primitives_combination(total_tip_dimension = [0.7, 0.3], \
#                                                                                     tf_tool_tip = (0, 0, 0.3, 0, 0, 0, 1),\
#                                                                                     max_suction_force = 150.0 , partition = [1, 2])

# ## 第 3 步:生成当前tool下吸盘上所有的tip_primitives， 并且将可以形成吸盘的tip_primitive进行组合
# ##         如果第一步不满足条件，需要取消下面代码注释完成该步骤，根据吸盘自行添加tip_primitive个数
# # tip_primitive_1 = bulid_tip_primitive(tool_name = tool_name, id="tp_1", tf_tool_region=(0.7/4, 0, 0.3, 0, 0, 0, 1), dimension=[0.35, 0.3], max_suction_force=150)
# # tip_primitive_2 = bulid_tip_primitive(tool_name = tool_name, id="tp_2", tf_tool_region=(-0.7/4, 0, 0.3, 0, 0, 0, 1), dimension=[0.35, 0.3], max_suction_force=150)
# # all_tip_primitives = [tip_primitive_1, tip_primitive_2] #请列举出所有的tip_primitive
# # all_tip_primitives_combinations = [["tp_1"], ["tp_2"], ["tp_1", "tp_2"]] #请根据吸盘和项目需求自行组合

# ## 第 4 步:创建tool上的碰撞体(由多个几何体组成，至少包含吸盘上方的障碍物)
# ##         relative_pose为障碍物中心坐标系相对于origin坐标系的pose
# ##         当origin=Pose(0, 0, 0, 0, 0, 0, 1)时，可以理解为origin是法兰坐标系, 警告： 对于tool的碰撞体来说， 其origin必须是Pose(0, 0, 0, 0, 0, 0, 1)， 即使设定成其他也不起作用
# ##         有其他碰撞体时，取消下面的注释后继续生成碰撞体
# combined_tool_collision = PrimitiveGroup(name="tool_collision", origin=Pose(0, 0, 0, 0, 0, 0, 1))
# combined_tool_collision.primitives.append(GeometricPrimitive(type=GeometricPrimitive.BOX, dimensions=[0.7, 0.3, 0.3], relative_pose=Pose(0, 0, 0.15, 0, 0, 0, 1)))
# # combined_tool_collision.primitives.append(GeometricPrimitive(type=GeometricPrimitive.BOX, dimensions=[0.5, 0.4, 0.2], relative_pose=Pose(-0.5, -0.3, 0.1, 0, 0, 0, 1)))
# # combined_tool_collision.primitives.append(GeometricPrimitive(type=GeometricPrimitive.BOX, dimensions=[0.5, 0.4, 0.2], relative_pose=Pose(-0.5, -0.3, 0.1, 0, 0, 0, 1)))

# ## 第 5 步:生成tool
# ##         defined_tip_primitives_combinations和all_tip_primitives为第1步(或第2步)求得
# ##         defined_tool_collision为第3步求得
# tool_1 = build_tool(tool_name = tool_name, defined_tip_primitives_combinations = all_tip_primitives_combinations, \
#                                                             defined_tip_primitives = all_tip_primitives, tip_shrink_height = 0.015, \
#                                                             defined_tool_collision = combined_tool_collision)

# ## 第 6 步： 定义Tool坐标系（这里的tool坐标系一般意义上指的是工具上和法兰坐标系位置原点重合的地方）相对法兰（endflange）的pose(mounting_transform = tf_endflange_tool), 这里默认没有旋转
# ## 详细定义可以见https://161.189.84.82:8003/xyz-release-doc/ubuntu2004/motion/xyz-motion-base_0.8.0/env-manager-msg/msg/Tool.html
# tool_1.mounting_transform = Pose(0, 0, 0, 0, 0, 0, 1)
# ## 如果旋转45度
# # tool_1.mounting_transform = Pose(0, 0, 0, 0, 0, 0.383, 0.924)



# 将吸盘/夹具加入机器人模型
all_robots.tools.append(tool_1)
all_robots.tools.append(tool_2)
robot_tool_state = RobotToolState()
robot_tool_state.robot_id = "0"
robot_tool_state.tool_name = "tool1"
robot_tool_state.has_tool = True
robot_tool_state.has_attached_collision_object = False
all_robots.robot_tool_states.append(robot_tool_state)
all_robots_ros = AllRobotToolStatesRos.from_ros_msg(all_robots)

# 将工作环境所有内容写入并保存
all_robots_ros.to_json_file("env_manager")
#all_robots_ros.to_json_file("/home/xyz/xyz_app/projects/dapeng_station_0/rafcon/init_robot_states.json")
workspace_env.to_json_file("env_manager")
from xyz_motion import save_env_version
save_env_version("env_manager")

# 查看工作空间和吸盘的参考命令
#workspace_env.show()
# all_robots_ros.get_active_tool("0").show()
import ipdb;ipdb.set_trace()
