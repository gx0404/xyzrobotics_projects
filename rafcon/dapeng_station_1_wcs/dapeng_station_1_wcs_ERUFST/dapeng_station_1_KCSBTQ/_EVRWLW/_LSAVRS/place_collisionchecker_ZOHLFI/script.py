from xyz_motion import CollisionChecker
from xyz_motion import PlanningEnvironmentRos
from xyz_motion import AllRobotToolStatesRos
from xyz_env_manager.client import modify_tool
from xyz_env_manager.client import get_all_robot_tool_states
from xyz_env_manager.client import get_planning_environment
from xyz_motion import ToolRos
import copy
from xyz_motion import RobotDriver,SE3,RobotRos
from py_xyz_mf import MovementFlow
import json
from xyz_motion import pose_to_list
from xyz_env_manager.msg import GeometricPrimitive
from xyz_env_manager.msg import Pose

def execute(self, inputs, outputs, gvm):
    self.logger.info("Hello {}".format(self.name))
    
    if not gvm.get_variable("motion_payload", per_reference=True, default=None):
        planning_env_msg = get_planning_environment()
        planning_env = PlanningEnvironmentRos.from_ros_msg(planning_env_msg)
    else:
        last_payload = gvm.get_variable("motion_payload", per_reference=True, default=None)
        planning_env = PlanningEnvironmentRos.from_ros_msg(last_payload["planning_environment"])

    #因为通过这个模块tool的初始碰撞体会被不停改变,所以做环境的时候注册一个初始化的tool环境文件   
    #all_robot_states = AllRobotToolStatesRos.from_json_file("/home/xyz/xyz_app/projects/dapeng_station_0/rafcon/init_robot_states.json")
    
    all_robot_states_msg = get_all_robot_tool_states()
    all_robot_states = AllRobotToolStatesRos.from_ros_msg(all_robot_states_msg)
    
    #得到初始的夹具
    check_our_robot = all_robot_states.get_robot_ros("0")
    check_our_robot.detach_object()
    kinematic_solver = check_our_robot.get_kinematics_solver()
    
    robot_tool_state = all_robot_states.get_robot_tool_state("0")
    tool_name = robot_tool_state.tool_name
    init_cur_tool = all_robot_states.get_tool_ros(tool_name).to_ros_msg()
    
    grasp_plan = inputs["grasp_plan"]

    #获取复用放置点
    update_object_poses = gvm.get_variable("object_poses", per_reference=False, default=None)
    if update_object_poses is None:
        raise "获取抓取姿态全局变量为空，查看初始化模块"
    else:
        object_poses = copy.copy([update_object_poses[0]])
        update_object_poses.pop(0)
        gvm.set_variable("object_poses", update_object_poses, per_reference=True)    

    q0 = inputs["q0"]
    if q0:
        init_joints = q0[0]
    else:
        raise "参考点必须连线"
    #放置偏置更新
    object_poses[0][2]-=0.035      
    init_joints_list = []
    init_joints = [0.0, -1.57, 1.4, 0.0]
    init_joints_list.append(init_joints)
    init_joints_list.append(init_joints[0:-1]+[init_joints[-1]+3.1415926])
    init_joints_list.append(init_joints[0:-1]+[init_joints[-1]-3.1415926])    
    #计算所有放置姿态法兰joint,tf_flange_obj是从抓取计算那里反算的
    if grasp_plan and object_poses:
        check_joints = []
        tf_flange_object = SE3(inputs["tf_flange_object"])
        
        for object_pose in object_poses:
            tf_map_flange = SE3(object_pose)*(tf_flange_object.inv())
            for init_joint in init_joints_list:
                check_joint = kinematic_solver.compute_best_ik(tf_map_flange, init_joint)
                if len(check_joint)==4:
                    check_joint = kinematic_solver.convert_four_dof_to_six(check_joint)
                    check_joints.append(check_joint) 
                    break
                else:
                    continue
                
    else:
        raise "抓取姿态和抓取规划必须连线"         
             
    if not check_joints:   
        self.logger.info("放置点ik无解")                     
        return "fail"  
                            
    """
    根据tool 生成需要判断的挡板碰撞体
    tool0:400*300*230  

    """
    
    #通过碰撞体名字找到对应碰撞体
    init_tool_collisions = []
    init_clamp_collision = []
    tool_check_collision_list = ["+y","-y","x"]
    for i in init_cur_tool.tool_collisions_list:
        if i.name in tool_check_collision_list:
            init_tool_collisions.append(i)
        else:
            init_clamp_collision.append(i)            

    #our robot
    our_robot_msg = check_our_robot.to_ros_msg()

    tf_flange_jonits = []
    #检测所有的姿态是否满足气缸碰撞条件，如果满足只取一组解

    for joint_index,check_joint in enumerate(check_joints):
        self.logger.info(f"开始 {check_joint} 检测碰撞")
        un_clamp_collision = []  
        un_clamp_collision_name = []  

        #通过放置姿态，遍历每一个气缸碰撞体是否碰撞
        for index,check_collision in enumerate(init_tool_collisions):
            #初始化机器人msg
            check_our_robot_msg = copy.copy(our_robot_msg)
            #添加默认就有的干涉物，像相机，夹具本体
            check_our_robot_msg.tool.tool_collisions.primitives=[i.primitives[0] for i in init_clamp_collision] 

            if tool_check_collision_list[index]!=check_collision.name:
                raise "气缸挡板碰撞体名字不匹配"
            check_our_robot_msg.tool.tool_collisions.primitives.append(check_collision.primitives[0])
 
            check_our_robot = RobotRos.from_ros_msg(check_our_robot_msg)
            # Set up collision checker
            checker = CollisionChecker(check_our_robot, planning_env)
            if checker.check_point_collision(check_joint):
                self.logger.info(f"检测到挡板{check_collision.name}与料箱干涉")         
            else:
                un_clamp_collision.append(check_collision)
                un_clamp_collision_name.append(check_collision.name)

        outputs["un_clamp_collision"] = un_clamp_collision
        outputs["un_clamp_collision_name"] = un_clamp_collision_name   
          
        #检测碰撞的气缸是否满足条件   
        if len(un_clamp_collision_name)<2 or "x" not in un_clamp_collision_name:
            if len(un_clamp_collision_name)<2:
                self.logger.info("此tool挡板下降不能小于两个")
            if "x" not in un_clamp_collision_name:
                self.logger.info("此tool挡板下降不能没有x")
            continue
        else:
            tf_flange_jonits.append(check_joint)
            continue        

    if not tf_flange_jonits:
        return "fail"        

    #添加一个放置姿态的机器人
    our_robot_msg.tool.tool_collisions.primitives = []
    our_robot_msg.tool.tool_collisions.primitives.append(GeometricPrimitive(type=GeometricPrimitive.BOX, dimensions=[0.55, 0.3, 0.2], relative_pose=Pose(-0.352-0.209/2, 0, 0.0835, 0, 0, 0.707, 0.707)))
    our_robot_msg.tool.tool_collisions.primitives.append(GeometricPrimitive(type=GeometricPrimitive.BOX, dimensions=[0.70-0.08, 0.5-0.06, 0.7], relative_pose=Pose(0, 0, 0.08+0.7/2+0.07, 0, 0, 0, 1)))
    for clamp_collision in un_clamp_collision:
        our_robot_msg.tool.tool_collisions.primitives.append(clamp_collision.primitives[0]) 
    for clamp_collision in init_clamp_collision:
        our_robot_msg.tool.tool_collisions.primitives.append(clamp_collision.primitives[0])         
    gvm.set_variable("our_robot_msg", our_robot_msg, per_reference=True)         

    tf_flange_jonits = list(map(lambda x:kinematic_solver.convert_six_dof_to_four(x),tf_flange_jonits))
                  
    
    tf_base_flange_list = list(map(kinematic_solver.compute_fk,tf_flange_jonits)) 
    object_poses = []
    for tf_base_flange in tf_base_flange_list:
        tf_base_object = tf_base_flange*tf_flange_object  
        object_poses.append(tf_base_object.xyz_quat)   
    #放置偏置更新
    object_poses[0][2]+=0.035                                                                                                          
    outputs["object_poses"] = object_poses   
    return "success"
