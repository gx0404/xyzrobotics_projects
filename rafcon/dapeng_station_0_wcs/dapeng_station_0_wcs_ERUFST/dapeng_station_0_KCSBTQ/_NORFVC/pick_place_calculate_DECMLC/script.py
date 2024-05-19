

from xyz_motion import PlanningEnvironmentRos,RobotDriver
from xyz_motion import AllRobotToolStatesRos,ToolRos,RobotRos
from xyz_env_manager.client import get_all_robot_tool_states
from xyz_env_manager.client import get_planning_environment
from xyz_motion import CollisionChecker,SE3,RobotRos,PlanningEnvironmentRos,pose_to_list
import tf.transformations as tfm
import numpy as np
import dill,time,copy
import multiprocessing
from queue import PriorityQueue
   
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
    # max_z = max(i.origin.z for i in items)
    # new_items = list(filter(lambda x:abs(x.origin.z-max_z)<0.1,items))
    return new_items

#过滤得到底层箱子
def filter_bottom_items(items):

    combined_data = {}
    for item in items:
        #建立x,y坐标的键，同一列箱子xy坐标一致
        key = (round(item.origin.x,2), round(item.origin.y,2))
        if key not in combined_data.keys():
            combined_data[key] = item
        else:   
            # 只保留Z最小的类实例
            if item.origin.z < combined_data[key].origin.z:
                combined_data[key] = item

    new_items = list(combined_data.values())
    return new_items


#后验
def check_pick_slide_move(checker,kinematic_solver,joints):
    use_slide = None
    slide_list = [[0.052, 0.052, 0.15, 0, 0, 0, 1], [0.052, -0.052, 0.15, 0, 0, 0, 1], [-0.052, 0.052, 0.15, 0, 0, 0, 1], [-0.052, -0.052, 0.15, 0, 0, 0, 1]]
    joints = kinematic_solver.convert_six_dof_to_four(joints)
    tf_map_flange_list_out = kinematic_solver.compute_fk(joints)
    for slide in slide_list:
        tf_map_flange_silde = SE3(slide)*tf_map_flange_list_out
        slide_joint = kinematic_solver.compute_best_ik(tf_map_flange_silde, joints)
        if not slide_joint:
            continue        
        slide_joint = kinematic_solver.convert_four_dof_to_six(slide_joint)
        if checker.check_point_collision(slide_joint):
            continue
        else:
            use_slide = slide 
            break
    if not use_slide:
        return False
    else:
        return True   

#后验
def check_place_slide_move(checker,kinematic_solver,joints):
    use_slide = None
    slide_list = [[0.042, 0.042, 0.1, 0, 0, 0, 1], [0.052, -0.052, 0.1, 0, 0, 0, 1], [-0.052, 0.052, 0.1, 0, 0, 0, 1], [-0.052, -0.052, 0.1, 0, 0, 0, 1]]
    joints = kinematic_solver.convert_six_dof_to_four(joints)
    tf_map_flange_list_out = kinematic_solver.compute_fk(joints)
    for slide in slide_list:
        tf_map_flange_silde = SE3(slide)*tf_map_flange_list_out
        slide_joint = kinematic_solver.compute_best_ik(tf_map_flange_silde, joints)
        if not slide_joint:
            continue        
        slide_joint = kinematic_solver.convert_four_dof_to_six(slide_joint)
        if checker.check_point_collision(slide_joint):
            continue
        else:
            use_slide = slide 
            break
    if not use_slide:
        return False
    else:
        return True   

class CostItem:

    def __init__(self, f_cost,path,state_key):
        self.f_cost = f_cost
        self.path = path
        self.state_key = state_key
    def __lt__(self, other):
        return self.f_cost < other.f_cost
               
class SEARCH_ASYNC():

    def __init__(self) -> None:

        #缓存来存储已计算的启发式代价
        self.heuristic_cache = {}

        # 初始化一个字典用于存储已访问的状态
        self.visited_states = {}
        
        # 进程计算队列
        self.calculation_queue = None
        # 进程终止事件
        self.preempted = None
        # 日志
        self.logger = None
        #垛型列数
        self.row = None

    def check_pick_collision(self,our_robot,check_robot_list,planning_env,init_joints,container_item,pick_path_key):
        #获取当前抓取箱子坐标
        tf_map_object = SE3(pose_to_list(container_item.origin))
        
        #获取抓取法兰所有姿态
        tf_tip_object_list = [[0,0,0,1,0,0,0],[0,0,0,0,1,0,0]]   
        tf_map_flange_list = []
        for tf_tip_object in tf_tip_object_list:
            append_list = []
            tf_tip_object = SE3(tf_tip_object)
            tf_map_flange = tf_map_object * tf_tip_object.inv() * our_robot.get_active_tool().get_tf_endflange_tip("tip_0").inv()
            #方便后续放置计算
            tf_flange_tip = our_robot.get_active_tool().get_tf_endflange_tip("tip_0")
            tf_flange_obj = tf_flange_tip*tf_tip_object

            append_list.append(tf_map_flange.xyz_quat)
            append_list.append(tf_flange_obj)

            tf_map_flange_list.append(append_list)   
      
        kinematic_solver = our_robot.get_kinematics_solver()

        #计算所有抓取姿态法兰joint
        check_joints = []
        init_jonints_list = []
        init_joints = [0.0, -1.57, 1.4, 0.0]
        init_jonints_list.append(init_joints)
        init_jonints_list.append(init_joints[0:-1]+[init_joints[-1]+3.1415926])
        init_jonints_list.append(init_joints[0:-1]+[init_joints[-1]-3.1415926])
        for i in tf_map_flange_list:
            for init_joint in init_jonints_list:
                append_list = []
                check_joint = kinematic_solver.compute_best_ik(SE3(i[0]), init_joint)
                if not check_joint:
                    continue
                else:
                    if len(check_joint)==4:
                        check_joint = kinematic_solver.convert_four_dof_to_six(check_joint)
                        append_list.append(check_joint)
                        append_list.append(i[1])
                        check_joints.append(append_list) 
                        break
                    else:  
                        append_list.append(check_joint)
                        append_list.append(i[1])
                        check_joints.append(append_list) 
                        break 
        if not check_joints:                
            return True,[]   
        #检测所有的姿态是否满足气缸碰撞条件，如果满足只取一组解
        collision_flag = True
        pick_joints = []
        for joint_index,check_joint in enumerate(check_joints):
            append_list = []
            for check_robot in check_robot_list:
            #添加碰撞检测器
                checker = CollisionChecker(check_robot, planning_env)   
                if checker.check_point_collision(check_joint[0]):
                    if pick_path_key==8:
                        pass
                        #import ipdb;ipdb.set_trace() 
                    continue
                else:
                    # if container_item.additional_info.values[-3]=="24":
                    #     pass
                        #import ipdb;ipdb.set_trace()
                    collision_flag = False     
                    pick_joints.append(check_joint)                          
                    break    
                    # if check_pick_slide_move(checker,kinematic_solver,check_joint[0]): 
                    #     collision_flag = False     
                    #     pick_joints.append(check_joint)    
                    #     break    
                    # else:
                    #     continue         
        return collision_flag,pick_joints

    def check_place_collision(self,our_robot,check_robot_list,planning_env,pick_joints,plan_item,pick_path_key):
      
        kinematic_solver = our_robot.get_kinematics_solver()
        for pick_joint in pick_joints:
            collision_flag = True
            init_joints = kinematic_solver.convert_six_dof_to_four(pick_joint[0])
            tf_flange_object = pick_joint[1]
            #获取当前放置箱子坐标
            tf_map_object = SE3(pose_to_list(plan_item.origin))
            tf_map_object = tf_map_object*SE3([0,0,-0.035,0,0,0,1])
            #获取放置姿态
            tf_map_flange = tf_map_object*(tf_flange_object.inv())
            #将抓取点作为初始点
            init_joints_list = []
            init_joints = [0.0, -1.57, 1.4, 0.0]
            init_joints_list.append(init_joints)
            init_joints_list.append(init_joints[0:-1]+[init_joints[-1]+3.1415926])
            init_joints_list.append(init_joints[0:-1]+[init_joints[-1]-3.1415926])  
            #放置点转化为joint 
            check_joints = []
            for init_joint in init_joints_list:
                check_joint = kinematic_solver.compute_best_ik(SE3(tf_map_flange), init_joint)
                if not check_joint:
                    continue
                else:
                    if len(check_joint)==4:
                        check_joint = kinematic_solver.convert_four_dof_to_six(check_joint)
                        check_joints.append(check_joint) 
                        break
                    else:  
                        check_joints.append(check_joint) 
                        break 
            if not check_joints:                
                continue  
            #检测所有的姿态是否满足气缸碰撞条件，如果满足只取一组解
            for joint_index,check_joint in enumerate(check_joints):
                for check_robot in check_robot_list:
                    #添加碰撞检测器
                    checker = CollisionChecker(check_robot, planning_env)   
                    if checker.check_point_collision(check_joint):
                        # if plan_item.additional_info.values[-3]=="25":
                        #     pass
                        #     import ipdb;ipdb.set_trace()
                        continue
                    else:
                        if pick_path_key==8:
                            pass
                            #import ipdb;ipdb.set_trace()
                        self.logger(f"放置开始后验,{plan_item.additional_info.values[-3]}")    
                        # collision_flag = False
                        # break    
                        if check_place_slide_move(checker,kinematic_solver,check_joint):     
                            collision_flag = False   
                            break  
                        else:
                            #import ipdb;ipdb.set_trace()                            
                            continue 
            #判断是否碰撞        
            if not collision_flag:            
                return_collision_flag = False
                break
            else:
                continue
        if collision_flag:
            return_collision_flag = True        
        #判断放置姿态如果旋转180能否放置    
        for pick_joint in pick_joints:
            collision_flag = True
            init_joints = kinematic_solver.convert_six_dof_to_four(pick_joint[0])
            tf_flange_object = pick_joint[1]

            #获取当前放置箱子坐标
            tf_map_object = SE3(pose_to_list(plan_item.origin))
            tf_map_object = tf_map_object*SE3([0,0,-0.035,0,0,0,1])
            #旋转放置箱子180
            tf_map_object = tf_map_object*SE3([0,0,0,0,0,1,0])
            #获取放置姿态
            tf_map_flange = tf_map_object*(tf_flange_object.inv())
            #将抓取点作为初始点
            init_joints_list = []
            init_joints_list.append(init_joints)
            init_joints_list.append(init_joints[0:-1]+[init_joints[-1]+3.1415926])
            init_joints_list.append(init_joints[0:-1]+[init_joints[-1]-3.1415926])  
            #放置点转化为joint 
            check_joints = []
            for init_joint in init_joints_list:
                check_joint = kinematic_solver.compute_best_ik(SE3(tf_map_flange), init_joint)
                if not check_joint:
                    continue
                else:
                    if len(check_joint)==4:
                        check_joint = kinematic_solver.convert_four_dof_to_six(check_joint)
                        check_joints.append(check_joint) 
                        break
                    else:  
                        check_joints.append(check_joint) 
                        break 
            if not check_joints:                
                continue   
            #检测所有的姿态是否满足气缸碰撞条件，如果满足只取一组解
            for joint_index,check_joint in enumerate(check_joints):
                for check_robot in check_robot_list:
                    #添加碰撞检测器
                    checker = CollisionChecker(check_robot, planning_env)   
                    if checker.check_point_collision(check_joint):
                        if pick_path_key==8:
                            pass
                            #import ipdb;ipdb.set_trace()
                        continue
                    else:
                        if pick_path_key==8:
                            pass
                            #import ipdb;ipdb.set_trace()
                        self.logger(f"放置开始后验180,{plan_item.additional_info.values[-3]}")        
                        # collision_flag = False 
                        # break
                        if check_place_slide_move(checker,kinematic_solver,check_joint):     
                            collision_flag = False   
                            break  
                        else:
                            continue               
        #如果旋转180°后还是检测到碰撞,说明无法放置；未检测到碰撞则是说明需要转下条码朝向                   
        if collision_flag:
            return_collision_flag_180 = False
        else:
            return_collision_flag_180 = True    
        return return_collision_flag,return_collision_flag_180  

    def cost_estimate(self,plan_item,cost_z,pick_box_id_row,from_pick_id):
        #还原原先顺序的代价
        if plan_item.additional_info.values[-3]==from_pick_id:
            from_cost = 0
        else:
            from_cost = 20     
               
        #还原回原来那一列的代价
        if int(plan_item.additional_info.values[-3])%self.row==pick_box_id_row:
            row_cost = 0
        else:
            row_cost = 50  
                      
        #填充高度的代价
        #z_cost = round((plan_item.origin.z - cost_z)/0.23)
        z_cost = 0
        return z_cost+row_cost+from_cost

    def search_path(self,our_robot,check_robot_list,planning_env,init_joints,cache_pallet_tote_data,\
                    plan_items,container_items,old_path,pick_path,pick_path_key):
        all_container_items = planning_env.get_container_items("1") 
        if len(old_path)==len(all_container_items):
            self.logger(f"上段路径已搜索结束")
            return old_path
        pick_path_list = []
        plan_path_list = []
        for i in old_path:
            pick_path_list.append(i[0])
            plan_path_list.append(i[1])
        #初始化环境   
        init_planning_env = copy.copy(planning_env)   

        #添加已拣配托盘放置箱子环境
        init_planning_env.add_container_items("0", plan_path_list) 
        #清除当前路径下缓存区箱子环境
        init_planning_env.clear_container_items("1", [i.name for i in pick_path_list]) 

        plan_items = list(filter(lambda x: x.name not in [i.name for i in plan_path_list], plan_items))
        
        remaining_plan_items = list(filter(lambda x:x.additional_info.values[-3] in pick_path,plan_items))
        remaining_plan_items = filter_bottom_items(remaining_plan_items)

        remaining_pick_items = list(filter(lambda x: x.name not in [i.name for i in pick_path_list], container_items))
        remaining_pick_items = filter_layer_items(remaining_pick_items)                 
        # 初始化优先队列
        open_set = PriorityQueue() 
        visited_states = {}
        all_path = [] 
        for pick_item in remaining_pick_items:
         # 判断箱子是否抓取干涉
            pick_collision_flag,pick_joints = self.check_pick_collision(our_robot, check_robot_list, init_planning_env, init_joints, pick_item,pick_path_key) 
            if pick_collision_flag:
                continue
            else:  
                #通过数据库找到之前在拣配箱的位置号
                pick_box_id = pick_item.additional_info.values[-3]
                from_pick_id = cache_pallet_tote_data[pick_box_id]["from_pick_id"]
                #原先哪一列
                pick_box_id_row = int(from_pick_id)%self.row
                
                for plan_item in remaining_plan_items:
                    #获取到放置是否干涉，以及旋转180放置是否干涉
                    place_collision_flag,change_180_no_collision = self.check_place_collision(our_robot,check_robot_list,init_planning_env,pick_joints,plan_item,pick_path_key) 
               
                    if not place_collision_flag:
                        cost_item = CostItem(self.cost_estimate(plan_item,0,pick_box_id_row,from_pick_id),old_path+[[pick_item,plan_item,False]],[plan_item.name]+[i.name for i in plan_path_list]) 
                        open_set.put(cost_item)
                    if  change_180_no_collision:
                        cost_item = CostItem(self.cost_estimate(plan_item,0,pick_box_id_row,from_pick_id),old_path+[[pick_item,plan_item,True]],[plan_item.name]+[i.name for i in plan_path_list])        
                        open_set.put(cost_item)             
                              
        #建立代价
        cost_z = None
        for plan_item in plan_items:
            current_z = plan_item.origin.z
            if not cost_z or current_z<cost_z:
                cost_z = current_z  

        while not open_set.empty() and not self.preempted:
            retrieved_item = open_set.get()
            f_cost = retrieved_item.f_cost
            path = retrieved_item.path
            state_key = retrieved_item.state_key
            if len(path)==len(pick_path)+len(old_path) or len(path)==len(all_container_items):
                self.logger(f"已找到可以还原的路径")           
                path_log = []
                for pick_place_plan in path:
                    place_box_id = pick_place_plan[1].additional_info.values[-3]
                    path_log.append(place_box_id)      
                self.logger(f"计算结果{path_log}中有拣配任务目标箱子的填充")
                all_path = path
                break

            #将路径节点ID当成键值
            current_state_key = tuple(sorted(state_key))
            # 判断是否已经访问过这个状态
            if current_state_key in visited_states:          
                continue   
            # 将当前状态添加到已访问的字典中
            visited_states[current_state_key] = "test"    

            pick_path_list = []
            plan_path_list = []
            for i in path:
                pick_path_list.append(i[0])
                plan_path_list.append(i[1])
            #初始化环境   
            check_planning_env = copy.copy(planning_env)   

            #添加已拣配托盘放置箱子环境
            check_planning_env.add_container_items("0", plan_path_list) 
            #清除当前路径下缓存区箱子环境
            check_planning_env.clear_container_items("1", [i.name for i in pick_path_list]) 
            # if len(path)>5:
            #     import ipdb;ipdb.set_trace()
            #过滤掉缓存区已完成的箱子
            remaining_pick_items = list(filter(lambda x:x.name not in [i.name for i in pick_path_list],container_items))
            #过滤缓存区非顶层箱子+最高层
            remaining_pick_items = filter_layer_items(remaining_pick_items)

            #过滤掉拣配托盘已放置的箱子
            remaining_plan_items = list(filter(lambda x:x not in plan_path_list,plan_items))
            remaining_plan_items = list(filter(lambda x:x.additional_info.values[-3] in pick_path,remaining_plan_items))
            #if pick_path_key==8:
                #import ipdb;ipdb.set_trace() 
            for pick_item in remaining_pick_items:
                #判断箱子是否抓取干涉
                pick_collision_flag,pick_joints = self.check_pick_collision(our_robot, check_robot_list, check_planning_env, init_joints, pick_item,pick_path_key)    
                if pick_collision_flag:
                    continue
                else:  
                    #通过数据库找到之前在拣配箱的位置号
                    pick_box_id = pick_item.additional_info.values[-3]
                    from_pick_id = cache_pallet_tote_data[pick_box_id]["from_pick_id"]
                    #原先哪一列
                    pick_box_id_row = int(from_pick_id)%self.row
                    #找到需要放置的箱子
                    new_remaining_plan_items = filter_bottom_items(remaining_plan_items)

                    for plan_item in new_remaining_plan_items:
                        #获取到放置是否干涉，以及旋转180放置是否干涉
                        place_collision_flag,change_180_no_collision = self.check_place_collision(our_robot,check_robot_list,check_planning_env,pick_joints,plan_item,pick_path_key) 
                        
                        if not place_collision_flag:
                            new_state_key = state_key + [plan_item.name]
                            new_path = copy.copy(path)
                            new_path.append([pick_item,plan_item,False])
                            cost_item = CostItem(f_cost+self.cost_estimate(plan_item,cost_z,pick_box_id_row,from_pick_id),new_path,new_state_key) 
                            open_set.put(cost_item)

                        if change_180_no_collision:
                            new_state_key = state_key + [plan_item.name]
                            new_path = copy.copy(path)
                            new_path.append([pick_item,plan_item,True])   
                            cost_item = CostItem(f_cost+self.cost_estimate(plan_item,cost_z,pick_box_id_row,from_pick_id),new_path,new_state_key) 
                            open_set.put(cost_item)
        # if not all_path:
        #     raise "fault"                    
        return all_path                    
             
                              

def execute(self, inputs, outputs, gvm):
    self.logger.info("Hello {}".format(self.name))
    planning_env_msg = get_planning_environment()
    planning_env = PlanningEnvironmentRos.from_ros_msg(planning_env_msg)

    #记录计算的次数
    global check_collision_num 
    check_collision_num = 0
    #计算所有抓取姿态法兰joint
    q0 = inputs["q0"]
    if q0:
        init_joints = q0
    else:
        self.logger.info("未收到input q0 通过机器人获取实时init joint")
        r = RobotDriver(0)
        init_joints = list(r.get_joints()) 

    all_robot_states = AllRobotToolStatesRos.from_ros_msg(get_all_robot_tool_states()) 
    our_robot = all_robot_states.get_robot_ros("0")
    our_robot.detach_object()      
    #得到初始的夹具 
    robot_tool_state = all_robot_states.get_robot_tool_state("0")
    tool_name = robot_tool_state.tool_name
    init_cur_tool = all_robot_states.get_tool_ros(tool_name).to_ros_msg()   

    #通过碰撞体名字找到对应碰撞体
    init_tool_collisions = []
    init_clamp_collision = []
    tool_check_collision_list = ["+y","-y","x"]
    for i in init_cur_tool.tool_collisions_list:
        if i.name in tool_check_collision_list:
            init_tool_collisions.append(i)
        else:
            init_clamp_collision.append(i)  
            
   #相机扩大一点
    for i in init_clamp_collision:
        if i.name == "camera":
            i.primitives[0].dimensions = [0.55, 0.3+0.03, 0.2]
        if i.name == "body":
            i.primitives[0].dimensions = [0.38, 0.58, 0.65]    
                   
    #手动设置夹具两种姿态，这个是为了算碰撞避免每次set tool
    #共有 x+y和x-y的两种our robot    
    check_robot_list = []   
    
    #x,+y our robot
    check_our_robot = all_robot_states.get_robot_ros("0") 
    check_our_robot.detach_object()   
    check_our_robot_msg = check_our_robot.to_ros_msg()

    #添加默认就有的干涉物，像相机，夹具本体
    check_our_robot_msg.tool.tool_collisions.primitives=[i.primitives[0] for i in init_clamp_collision] 

    #添加可活动气缸
    for index,clamp_collision in enumerate(init_tool_collisions):
        if clamp_collision.name =="x":
            check_our_robot_msg.tool.tool_collisions.primitives.append(clamp_collision.primitives[0])
        if clamp_collision.name == "+y":
            check_our_robot_msg.tool.tool_collisions.primitives.append(clamp_collision.primitives[0])

    #因为our robot 不能用copy模块序列化，所以需要从新序列化        
    check_robot_list.append(RobotRos.from_ros_msg(check_our_robot_msg))

    #x,-y our robot
    check_our_robot = all_robot_states.get_robot_ros("0") 
    check_our_robot.detach_object()   
    check_our_robot_msg = check_our_robot.to_ros_msg()
    #添加默认就有的干涉物，像相机，夹具本体
    check_our_robot_msg.tool.tool_collisions.primitives=[i.primitives[0] for i in init_clamp_collision] 
    #添加可活动气缸
    for index,clamp_collision in enumerate(init_tool_collisions):
        if clamp_collision.name =="x":
            check_our_robot_msg.tool.tool_collisions.primitives.append(clamp_collision.primitives[0])
        if clamp_collision.name == "-y":
            check_our_robot_msg.tool.tool_collisions.primitives.append(clamp_collision.primitives[0])

    #因为our robot 不能用copy模块序列化，所以需要从新序列化        
    check_robot_list.append(RobotRos.from_ros_msg(check_our_robot_msg))


    #获取数据库缓存区物料信息，找到箱子之前从拣配托盘哪里抓的
    cache_pallet_tote_data = inputs["cache_pallet_tote_data"]

    #获取到之前拣配记录的抓取箱子
    restore_path = inputs["path"]
    new_pick_path = {}

    for key,items in restore_path.items():
        try:
            new_pick_path[int(key)]=items
        except ValueError:
            continue   

    sort_pick_path_key = sorted(new_pick_path)    

    #获取缓存区环境上的箱子
    container_items = planning_env.get_container_items("1")     
    #获取拣配托盘上放置规划箱子,并只得到最底层的规划
    plan_items = planning_env.get_unfinished_planned_items("0")

    search_async = SEARCH_ASYNC()
    search_async.preempted = self.preempted
    search_async.logger = self.logger.info
    search_async.row = gvm.get_variable("row", per_reference=False, default=None)

    path = []
    pick_path = []
    for pick_path_key in sort_pick_path_key:
        self.logger.info(f"开始搜索 {pick_path_key} 路径")
        pick_path_item = new_pick_path[pick_path_key]   
        direction = pick_path_item["direction"]
        pick_path += pick_path_item["pick_path"] 
    
    async_input = (our_robot, check_robot_list, planning_env, init_joints, \
            cache_pallet_tote_data, plan_items, container_items,\
            path,pick_path,pick_path_key)
    path = search_async.search_path(*async_input)

    #获取进程队列结果      
    all_paths = path
 
    if not all_paths:
        self.logger.info("no_path")
        return "no_path"                          
    else:
        return_path = []
        for pick_place_plan in all_paths:
            pick_box_id = pick_place_plan[0].additional_info.values[-3]
            place_box_id = pick_place_plan[1].additional_info.values[-3]
            return_path.append([pick_box_id,place_box_id,pick_place_plan[2]])   
        self.logger.info(f"all path is {return_path}")    
        gvm.set_variable("cache_restore_path", return_path, per_reference=False)
        gvm.set_variable("first_calculate", False, per_reference=False) 
        return "success"                   