from xyz_motion import CollisionChecker
from xyz_motion import PlanningEnvironmentRos
from xyz_motion import AllRobotToolStatesRos
from xyz_env_manager.client import modify_tool
from xyz_env_manager.client import get_all_robot_tool_states
from xyz_env_manager.client import get_planning_environment
from xyz_motion import ToolRos,pose_to_list,RobotRos
import copy
from xyz_motion import RobotDriver,SE3
from xyz_env_manager.msg import GeometricPrimitive
from xyz_env_manager.msg import Pose

#过滤得到顶层箱子
def filter_layer_items(items):

    combined_data = {}
    for item in items:
        #建立x,y坐标的键，同一列箱子xy坐标一致
        key = (round(item.origin.x,2), round(item.origin.y,2))
        if key not in combined_data.keys():
            combined_data[key] = item
        else:   
            # 只保留Z最大的类实例
            if item.origin.z > combined_data[key].origin.z:
                combined_data[key] = item

    new_items = list(combined_data.values())
    max_z = max(i.origin.z for i in items)
    new_items = list(filter(lambda x:abs(x.origin.z-max_z)<0.1,items))
    return new_items

def execute(self, inputs, outputs, gvm):
    self.logger.info("Hello {}".format(self.name))
    
    if not gvm.get_variable("motion_payload", per_reference=True, default=None):
        planning_env_msg = get_planning_environment()
        planning_env = PlanningEnvironmentRos.from_ros_msg(planning_env_msg)
    else:
        last_payload = gvm.get_variable("motion_payload", per_reference=True, default=None)
        planning_env = PlanningEnvironmentRos.from_ros_msg(last_payload["planning_environment"])
        
    
    #因为通过这个模块tool的初始碰撞体会被不停改变,所以做环境的时候注册一个初始化的tool环境文件
    all_robot_states_msg = get_all_robot_tool_states()
    all_robot_states = AllRobotToolStatesRos.from_ros_msg(all_robot_states_msg)
    
    #得到初始的夹具
    check_our_robot = all_robot_states.get_robot_ros("0")
    check_our_robot.detach_object()
    kinematic_solver = check_our_robot.get_kinematics_solver()
    robot_tool_state = all_robot_states.get_robot_tool_state("0")
    tool_name = robot_tool_state.tool_name
    init_cur_tool = all_robot_states.get_tool_ros(tool_name).to_ros_msg()
    
    
    #考虑到后续计算碰撞时会指定唯一解，所以抓取姿态从全局变量里依次排查
    update_tf_map_flange_list = gvm.get_variable("tf_map_flange_list", per_reference=False, default=None)
    
    if update_tf_map_flange_list is None:
        raise "获取抓取姿态全局变量为空，查看初始化模块"
    else:
        tf_map_flange_list = copy.copy([update_tf_map_flange_list[0]])
        update_tf_map_flange_list.pop(0)
        gvm.set_variable("tf_map_flange_list", update_tf_map_flange_list, per_reference=True)     
    grasp_plan = inputs["grasp_plan"]

    q0 = inputs["q0"]
    if q0:
        init_joints = q0
    else:
        self.logger.info("未收到input q0 通过机器人获取实时init joint")
        r = RobotDriver(0)
        init_joints = list(r.get_joints()) 

    #计算所有抓取姿态法兰joint
    init_joints_list = []
    #init_joints[-1] = 0
    init_joints = [0.0, -1.57, 1.4, 0.0]
    init_joints_list.append(init_joints)
    init_joints_list.append(init_joints[0:-1]+[init_joints[-1]+3.1415926])
    init_joints_list.append(init_joints[0:-1]+[init_joints[-1]-3.1415926])           

    if grasp_plan and tf_map_flange_list:
        check_joints = []        
        for tf_map_flange in tf_map_flange_list:
            for init_joint in init_joints_list:
                check_joint = kinematic_solver.compute_best_ik(SE3(tf_map_flange), init_joint)
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
        all_un_clamp_collision = []  
        all_un_clamp_collision_name = []  

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
                all_un_clamp_collision.append(check_collision)
                all_un_clamp_collision_name.append(check_collision.name)
                    
                    
        #检测碰撞的气缸是否满足条件     
        if len(all_un_clamp_collision_name)<2 or "x" not in all_un_clamp_collision_name:
            if len(all_un_clamp_collision_name)<2:
                self.logger.info("此tool挡板下降不能小于两个")
            if "x" not in all_un_clamp_collision_name:
                self.logger.info("此tool挡板下降不能没有x")
            if joint_index+1>=len(check_joints):
                return "fail"
            else:
                continue
        else:
            tf_flange_jonits.append(check_joint)
            break        
     

    tf_map_flange_list_out = []   
    
    if len(init_joints)==4:
        tf_flange_jonits = list(map(lambda x:kinematic_solver.convert_six_dof_to_four(x),tf_flange_jonits))
                                                                                                     
    for i in tf_flange_jonits:  
        tf_map_flange_list_out.append((kinematic_solver.compute_fk(i)).xyz_quat)
    outputs["tf_map_flange_list"] = tf_map_flange_list_out   
    outputs["place_init_joints"] = tf_flange_jonits
    
    #输出tf_flange_obj给放置计算使用
    tf_map_flange = SE3(tf_map_flange_list_out[0])
    for pg in grasp_plan.objects:
        if pg.name == grasp_plan.reference_object_name:
            tf_map_object = SE3(pose_to_list(pg.origin))
            break
    else:
        raise Exception("Cannot find reference_object_name: {} in grasp plan!".format(grasp_plan.reference_object_name))        
    tf_flange_object = ((tf_map_flange.inv())*tf_map_object).xyz_quat
    outputs["tf_flange_object"] = tf_flange_object 


    self.logger.info(f"抓取点未干涉的夹具为{all_un_clamp_collision_name}")
    if len(all_un_clamp_collision_name)==3:
        check_un_clamp_collision = []
        check_un_clamp_collision.append(all_un_clamp_collision)
        check_un_clamp_collision.append(list(filter(lambda x:x.name in ["x","+y"],all_un_clamp_collision)))
        check_un_clamp_collision.append(list(filter(lambda x:x.name in ["x","-y"],all_un_clamp_collision)))
    else:
        check_un_clamp_collision = [all_un_clamp_collision]        
        
        
    for un_clamp_collision in check_un_clamp_collision:
        un_clamp_collision_name = [i.name for i in un_clamp_collision]
        #选择检测使用偏移值
        slide_list = self.smart_data["slide_move"]   
        #添加一个抓取姿态的机器人
        our_robot_msg.tool.tool_collisions.primitives = []

        #缩小相机
        for i in init_clamp_collision:
            if i.name == "camera":
                i.primitives[0].dimensions = [0.55, 0.33, 0.2]
                        
        our_robot_msg.tool.tool_collisions.primitives=[i.primitives[0] for i in init_clamp_collision]
        for clamp_collision in un_clamp_collision:
            our_robot_msg.tool.tool_collisions.primitives.append(clamp_collision.primitives[0])     

        #检测哪个偏移点不干涉    
        check_slide_robot = RobotRos.from_ros_msg(our_robot_msg)
        attached_collision_object = gvm.get_variable("attached_collision_object", per_reference=False, default=None)
        attached_collision_object.origin_tip_transform = Pose(0,0,0,1,0,0,0)
        check_slide_robot.attach_object(attached_collision_object,[])
        planning_env.clear_container_items(grasp_plan.from_workspace_id,[grasp_plan.objects[0].name])
        
        check_slide_list = []
        
        for slide in slide_list:
            tf_map_flange_silde = SE3(slide)*SE3(tf_map_flange_list_out[0])
            init_joints_list = []
            init_joints_list.append(init_joints)
            init_joints_list.append(init_joints[0:-1]+[init_joints[-1]+3.1415926])
            init_joints_list.append(init_joints[0:-1]+[init_joints[-1]-3.1415926])   
            for init_joint in init_joints_list:
                slide_joint = kinematic_solver.compute_best_ik(tf_map_flange_silde, init_joint)
                if not slide_joint:
                    continue
                else:
                    break
            if not slide_joint:
                self.logger.info(f"计算偏移点{slide}IK无解")      
                continue       
            slide_joint = kinematic_solver.convert_four_dof_to_six(slide_joint)
            checker = CollisionChecker(check_slide_robot, planning_env)
            if checker.check_point_collision(slide_joint):
                self.logger.info(f"偏移点{slide}抓取碰撞")
                continue
            else:
                self.logger.info(f"偏移点{slide}抓取不碰撞")
                check_slide_list.append(slide)
                continue
        if not check_slide_list:
            #import ipdb;ipdb.set_trace()
            self.logger.info(f"夹具{un_clamp_collision_name}无法计算出非碰撞的slide")  
        self.logger.info(f"检测到非碰撞的check slide list 为{check_slide_list}")
        
        """后验"""
        pick_workspace_id = grasp_plan.from_workspace_id
        
        #拆分左右偏移精度
        def split_slide(slide,precision):
            split_x_length = int(slide[0]/precision)
            split_y_length = int(slide[1]/precision)
            new_slide_x = []
            new_slide_y = []
            for i in range(1,abs(split_x_length)+1):
                if split_x_length>0:
                    if i*precision>=0.012:
                        new_slide_x.append(i*precision)
                else:
                    if -i*precision<=-0.012:
                        new_slide_x.append(-i*precision)        
            for i in range(1,abs(split_y_length)+1):
                if split_y_length>0:
                    if i*precision>=0.012:
                        new_slide_y.append(i*precision)
                else:
                    if -i*precision<=-0.012:
                        new_slide_y.append(-i*precision)   
            return_list = [[x,y]+slide[2:7] for x,y in zip(new_slide_x,new_slide_y)]                               
            return return_list        
                    
            
                
        #偏移检测
        use_slide_list = []
        for check_slide in check_slide_list:
            # 指定精度
            precision = 0.001
            check_offset_xy_list = split_slide(check_slide,precision)
            self.logger.info(f"check_offset_xy_list is {check_offset_xy_list}")
            collision_flag = False
            for check_slide_xy in check_offset_xy_list:
                tf_map_flange_silde = SE3(check_slide_xy)*SE3(tf_map_flange_list_out[0])
                init_joints_list = []
                init_joints_list.append(init_joints)
                init_joints_list.append(init_joints[0:-1]+[init_joints[-1]+3.1415926])
                init_joints_list.append(init_joints[0:-1]+[init_joints[-1]-3.1415926])   
                for init_joint in init_joints_list:
                    check_slide_joint = kinematic_solver.compute_best_ik(tf_map_flange_silde, init_joint)
                    if not check_slide_joint:
                        continue
                    else:
                        break
                if not check_slide_joint:
                    self.logger.info(f"后验XY偏移点{check_slide_xy}IK无解")  
                    collision_flag = True
                    break       
                check_slide_joint = kinematic_solver.convert_four_dof_to_six(check_slide_joint)
                checker = CollisionChecker(check_slide_robot, planning_env)
                if checker.check_point_collision(check_slide_joint):
                    self.logger.info(f"后验XY偏移点{check_slide_xy}碰撞")
                    collision_flag = True
                    break   
            if not collision_flag:
                self.logger.info(f"后验xy偏移点{check_slide}未检测到碰撞")   
                use_slide_list.append(check_slide)
            else:
                self.logger.info(f"后验xy偏移点{check_slide}检测到碰撞")   
                    
        if not use_slide_list:
            self.logger.info(f"夹具{un_clamp_collision_name}后验XY偏移点检测无解")  
            continue
        self.logger.info(f"use slide list 为{use_slide_list}")               
        
        #通过现有最高的箱子计算偏移
        container_items = planning_env.get_container_items(pick_workspace_id)
        workspace_ros = planning_env.get_workspace_ros(pick_workspace_id)
        tf_flange_tip = SE3(pose_to_list(grasp_plan.tip.tip_pose))

        work_space_pose = workspace_ros.get_bottom_pose().xyz_quat
        tip_pose_z = tf_flange_tip.xyz_quat[2]
        sku_max_height = 0.3
        
        if container_items:
            check_items = filter_layer_items(container_items)
            space_pose_obj_z = check_items[0].origin.z+tip_pose_z+sku_max_height+0.2
        else:
            space_pose_obj_z = work_space_pose[2]+tip_pose_z+sku_max_height+0.2              
        if space_pose_obj_z<1.1:
            space_pose_obj_z = 1.1 
            offset_z = space_pose_obj_z
        else:     
            pose_base_flange = copy.deepcopy(tf_map_flange_list_out[0])
            offset_z = space_pose_obj_z- pose_base_flange[2]
            self.logger.info(f"offset_z为{offset_z}")
        
        z_precision = 0.03
            
        def divide_float_into_list(value, precision):
            # 计算需要多少步骤来从0达到目标值，加1是为了包含最终值本身
            steps = int(value / precision) + 1
            # 生成列表，使用round确保浮点数精度问题不会影响结果
            result = [round(i * precision, 2) for i in range(1, steps)]
            return result    
        
        offset_list = divide_float_into_list(offset_z,z_precision)
        self.logger.info(f"offset_list为{offset_list}")
        
        use_slide = None
        for use_slide_check in use_slide_list:
            offset_check_slide_list = []
            for offset in offset_list:
                append_check_slide = copy.deepcopy(use_slide_check)
                append_check_slide[2] = offset 
                offset_check_slide_list.append(append_check_slide)    
            self.logger.info(f"检查偏移点向上偏移,偏移点为{offset_check_slide_list}") 

            #后验检测
            self.logger.info(f"开始后验z方向碰撞检测")
            z_check_collision = False
            for check_slide in offset_check_slide_list:
                tf_map_flange_silde = SE3(check_slide)*SE3(tf_map_flange_list_out[0])
                init_joints_list = []
                init_joints_list.append(init_joints)
                init_joints_list.append(init_joints[0:-1]+[init_joints[-1]+3.1415926])
                init_joints_list.append(init_joints[0:-1]+[init_joints[-1]-3.1415926])   
                for init_joint in init_joints_list:
                    check_slide_joint = kinematic_solver.compute_best_ik(tf_map_flange_silde, init_joint)
                    if not check_slide_joint:
                        continue
                    else:
                        break
                if not check_slide_joint:
                    self.logger.info(f"后验Z偏移点IK无解")      
                    z_check_collision = True
                    break       
                check_slide_joint = kinematic_solver.convert_four_dof_to_six(check_slide_joint)
                checker = CollisionChecker(check_slide_robot, planning_env)
                if checker.check_point_collision(check_slide_joint):
                    self.logger.info(f"后验偏移点Z{check_slide}碰撞")
                    z_check_collision = True
                    break
            if z_check_collision:
                self.logger.info(f"后验偏移点Z{use_slide_check}无解")
                continue
            else:
                use_slide = use_slide_check
                self.logger.info(f"后验偏移点Z{use_slide_check}验证成功")
                break
        if not use_slide:
            self.logger.info(f"夹具{un_clamp_collision_name}后验偏移点Z无解")  
            continue
        else:
            self.logger.info(f"夹具{un_clamp_collision_name}后验成功")
            break
        
    if not use_slide:
        self.logger.info("无有效的抓取点")
        return "fail"  
             
    if use_slide[0]>0:
        use_slide[0] = 0.03
    else:
        use_slide[0] = -0.03    
    if use_slide[1]>0:
        use_slide[1] = 0.03
    else:
        use_slide[1] = -0.03   
    use_slide[2] = 0.03                   
    outputs["slide"] = [use_slide]  
    self.logger.info(use_slide)
    post_slide = copy.deepcopy(use_slide)
    post_slide[2] = -0.01
    outputs["post_slide"] = [post_slide]   
    outputs["un_clamp_collision"] = un_clamp_collision
    outputs["un_clamp_collision_name"] = un_clamp_collision_name  
    
    return "success"

