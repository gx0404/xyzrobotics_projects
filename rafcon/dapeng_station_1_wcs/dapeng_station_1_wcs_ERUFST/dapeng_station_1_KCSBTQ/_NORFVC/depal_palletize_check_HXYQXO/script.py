import sys
sys.path.append('/home/xyz/xyz_app/app/rafcon/')

from xyz_motion import PlanningEnvironmentRos,pose_to_list,SE3
from xyz_env_manager.client import get_planning_environment
import numpy as np
from xyz_env_manager.client import set_planned_items,clear_planned_items
from xyz_env_manager.client import clear_container_all_items,add_container_items
import tf.transformations as tfm
import copy
from xyz_env_manager.msg import Pose
from itertools import permutations
from itertools import product
import concurrent.futures
import threading
import queue
from depal_palletize_check import CALCU_PALLETIZE_ASYNC
import multiprocessing
import dill,time

class Node:
   def __init__(self, value):
      self.value = value
      self.next = None
      
class Stack:
   def __init__(self):
      self.top = None

   def push(self, value):
      new_node = Node(value)
      new_node.next = self.top
      self.top = new_node

   def pop(self):
      if self.top is None:
         return None
      popped_value = self.top.value
      self.top = self.top.next
      return popped_value

   def is_empty(self):
      return self.top is None
  
#过滤得到底层箱子
def filter_bottom_items(items,row_flag=True):
    #两种模式,一种只取最低层，另一种则是每列箱子的最低层
    if row_flag:
        combined_data = {}
        for item in items:
            #建立x,y坐标的键，同一列箱子xy坐标一致
            key = (round(item.origin.x,2), round(item.origin.y,2))
            if key not in combined_data.keys():
                #判断原先字典是否有xy近似的key的标志flag
                check_key_flag = False
                for check_key in combined_data.keys():
                    #判断绝对值是否小于0.015，如果xy都小于0.015，则认为是同列箱子
                    if abs(item.origin.x-check_key[0])<0.015 and abs(item.origin.y-check_key[1])<0.015:    
                        check_key_flag = True
                        break
                #如果不存在标志,则说明是个新列         
                if not check_key_flag:                    
                    combined_data[key] = item
                #如果存在,则说明是老列,则需要判断是否保留z最小的实例   
                else:
                    if item.origin.z < combined_data[check_key].origin.z:
                        combined_data[check_key] = item                  
            else:   
                # 只保留Z最小的类实例
                if item.origin.z < combined_data[key].origin.z:
                    combined_data[key] = item

        new_items = list(combined_data.values())
    #只考虑最低列,不考虑每列层数不同      
    else:    
        min_z = min(i.origin.z for i in items)
        new_items = list(filter(lambda x:abs(x.origin.z-min_z)<0.1,items))    
    return new_items

def all_direction_items(planning_env,container_items,current_direction):
    """
    获取所有转向的信息

    Parameters:
    - planning_env: 当前转向的环境
    - container_items: 当前转向的箱子信息
    - current_direction: 当前转向

    Returns:
    返回一个字典,字典key为转向,value为转向的环境,箱子
    """

    #agv托盘面对应的角度值，通过这个度数可以知道转到那个面需要转多少度
    if current_direction == "A":
        all_direction_angle = {
                "A":0,
                "B":-90,
                "C":180,
                "D":90
        }
    elif current_direction == "B":
        all_direction_angle = {
                "A":90,
                "B":0,
                "C":-90,
                "D":180
        }           
    elif current_direction == "C":
        all_direction_angle = {
                "A":180,
                "B":90,
                "C":0,
                "D":-90
        }    
    elif current_direction == "D":
        all_direction_angle = {
                "A":-90,
                "B":180,
                "C":90,
                "D":0
        } 

    all_direction_items_dict = {}

    #遍历所有面
    for direction,angle in all_direction_angle.items():
        #计算需要旋转的度数
        rotation_angle = angle

        #获取托盘旋转rotation 托盘坐标
        tf_space_rotation = SE3(tfm.quaternion_from_euler(0,0,np.deg2rad(rotation_angle)))
        tf_base_space = planning_env.get_workspace_ros("0").get_bottom_pose()    

        #初始箱子和环境
        new_container_items = copy.deepcopy(container_items)
        new_planning_env = copy.deepcopy(planning_env)

        #旋转箱子，是根据托盘坐标系左乘
        for container_item in new_container_items:
            tf_base_obj = SE3(pose_to_list(container_item.origin))
            tf_space_obj = (tf_base_space.inv())*tf_base_obj
            tf_new_space_obj = tf_space_rotation*tf_space_obj
            tf_new_base_obj = tf_base_space*tf_new_space_obj
            container_item.origin = Pose(*tf_new_base_obj.xyz_quat)

        #删除环境箱子，并添加新的箱子   
        new_planning_env.clear_container_all_items("0")
        new_planning_env.add_container_items("0",new_container_items)  
        
        all_direction_items_dict[direction] = {}
        all_direction_items_dict[direction]["container_items"] = new_container_items
        all_direction_items_dict[direction]["planning_env"] = new_planning_env

    return all_direction_items_dict   

class OBB:
    def __init__(self, center, half_size, angle):
        self.center = np.array(center)
        self.half_size = np.array(half_size)
        self.angle = angle
        self.axes = self.compute_axes()

    def compute_axes(self):
        cos_a = np.cos(self.angle)
        sin_a = np.sin(self.angle)
        u1 = np.array([cos_a, sin_a])      # 主轴（长边方向）
        u2 = np.array([-sin_a, cos_a])     # 副轴（短边方向）
        return u1, u2

    def get_corners(self):
        u1, u2 = self.axes
        corner1 = self.center + self.half_size[0] * u1 + self.half_size[1] * u2
        corner2 = self.center + self.half_size[0] * u1 - self.half_size[1] * u2
        corner3 = self.center - self.half_size[0] * u1 - self.half_size[1] * u2
        corner4 = self.center - self.half_size[0] * u1 + self.half_size[1] * u2
        return [corner1, corner2, corner3, corner4]

def project_obb(obb, axis):
    corners = obb.get_corners()
    projections = [np.dot(corner, axis) for corner in corners]
    return min(projections), max(projections)

def obb_overlap(obb1, obb2):
    axes = [obb1.axes[0], obb1.axes[1], obb2.axes[0], obb2.axes[1]]
    
    for axis in axes:
        min1, max1 = project_obb(obb1, axis)
        min2, max2 = project_obb(obb2, axis)
        
        if max1 < min2 or max2 < min1:
            return False
    
    return True

#通过obb计算是否碰撞
def is_collision(check_plan_item,other_plan_items):
    check_item_pose = pose_to_list(check_plan_item.origin)
    l_1,w_1,h_1 = check_plan_item.primitives[0].dimensions
    check_item_angle = tfm.euler_from_quaternion(check_item_pose[3:7])[2]
    check_item_obb = OBB(center=(check_item_pose[0], check_item_pose[1]), half_size=(l_1/2-0.00,w_1/2-0.00), angle=check_item_angle)

    for plan_item in other_plan_items:
        check_collision = True
        item_pose = pose_to_list(plan_item.origin)
        l_2,w_2,h_2 = plan_item.primitives[0].dimensions
        item_angle = tfm.euler_from_quaternion(item_pose[3:7])[2]
        item_obb = OBB(center=(item_pose[0], item_pose[1]), half_size=(l_2/2,w_2/2), angle=item_angle)
        
        if not obb_overlap(check_item_obb,item_obb):
            check_collision = False
        if check_collision:
            break
    return check_collision   

#计算合适的旋转角度
def get_rotate_angle(item,bottom_container_items):
    l,w,h = item.primitives[0].dimensions
    #旋转的箱子长边
    tfs_box_vertices = [[l/2, w/2, 0, 0, 0, 0, 1], [-l/2, w/2, 0, 0, 0, 0, 1]]
    tf_world_box = SE3(pose_to_list(item.origin))
    tfs_world_vertices = [(tf_world_box * SE3(tf)).xyz_quat for tf in tfs_box_vertices]
    #偏置的箱子长边方向向量
    A = np.array([tfs_world_vertices[1][0] - tfs_world_vertices[0][0], tfs_world_vertices[1][1] - tfs_world_vertices[0][1]])
    
    #拿到需要旋转的箱子距离最近的箱子
    bottom_container_items = sorted(bottom_container_items,key=lambda x:abs(x.origin.x-item.origin.x)+abs(x.origin.y-item.origin.y))
    near_item = bottom_container_items[0]
    #拿到最近的箱子长边方向向量
    l_2,w_2,h_2 = near_item.primitives[0].dimensions
    tfs_box_vertices = [[l_2/2, w_2/2, 0, 0, 0, 0, 1], [-l_2/2, w_2/2, 0, 0, 0, 0, 1]]
    tf_world_box = SE3(pose_to_list(near_item.origin))
    tfs_world_vertices = [(tf_world_box * SE3(tf)).xyz_quat for tf in tfs_box_vertices]
    B = np.array([tfs_world_vertices[1][0] - tfs_world_vertices[0][0], tfs_world_vertices[1][1] - tfs_world_vertices[0][1]])
    #print(f"旋转箱 {item.additional_info.values[-3]} ,参考箱子 {near_item.additional_info.values[-3]}")
    # 计算点积
    dot_product = np.dot(A, B)
    # 计算模长
    magnitude_A = np.linalg.norm(A)
    magnitude_B = np.linalg.norm(B)
    # 计算旋转角度
    cos_theta = dot_product / (magnitude_A * magnitude_B)
    angle_rad = np.arccos(cos_theta)  # 弧度+
    angle_deg = np.degrees(angle_rad) # 角度+
    #print("角度：", angle_deg)
    #垂直
    if angle_deg > 80 and angle_deg<100:
        if angle_deg>85 and angle_deg<95:
            rotate_rad = np.deg2rad(angle_deg-90)
        else:
            rotate_rad = np.deg2rad(5)
    #水平       
    elif angle_deg<10:
        if angle_deg<5:
            rotate_rad = np.deg2rad(angle_deg)
        else:
            rotate_rad = np.deg2rad(5) 
    #水平       
    elif angle_deg>170:
        if angle_deg>175:
            rotate_rad = np.deg2rad(180-angle_deg)
        else:
            rotate_rad = np.deg2rad(5)                            
    else:
        raise Exception("角度不在范围内")
    
    # 计算叉积,正数为逆时针，负数为顺时针
    cross_product_z = A[0] * B[1] - A[1] * B[0]    
    if cross_product_z>0:
        #相对逆时针,需要顺时针旋转
        tf_box_rotate = SE3(tfm.quaternion_from_euler(0,0,rotate_rad))
        #print(f"顺时针旋转{np.rad2deg(rotate_rad)}")
    else:
        #相对顺时针,需要逆时针旋转
        tf_box_rotate = SE3(tfm.quaternion_from_euler(0,0,-rotate_rad))      
        #print(f"逆时针旋转{np.rad2deg(-rotate_rad)}")  
    return tf_box_rotate
    

# #求角点坐标
# def get_rect_corners(item):
#     l,w,h = item.primitives[0].dimensions
#     l-=0.0015
#     w-=0.0015
#     tfs_box_vertices = [[l/2, w/2, 0, 0, 0, 0, 1], [-l/2, w/2, 0, 0, 0, 0, 1], [-l/2, -w/2, 0, 0, 0, 0, 1], [l/2, -w/2, 0, 0, 0, 0, 1]]
#     tf_world_box = SE3(pose_to_list(item.origin))
#     tfs_world_vertices = [(tf_world_box * SE3(tf)).xyz_quat for tf in tfs_box_vertices]
#     max_x = max([tf[0] for tf in tfs_world_vertices])
#     min_x = min([tf[0] for tf in tfs_world_vertices])
#     max_y = max([tf[1] for tf in tfs_world_vertices])
#     min_y = min([tf[1] for tf in tfs_world_vertices])
#     return max_x,min_x,max_y,min_y

# #AABB包装盒碰撞检测
# def is_collision(check_plan_item,other_plan_items):
#     max_x_1,min_x_1,max_y_1,min_y_1 = get_rect_corners(check_plan_item)
#     for plan_item in other_plan_items:
#         check_collision = True
#         max_x_2,min_x_2,max_y_2,min_y_2 = get_rect_corners(plan_item)
#         if min_x_1>max_x_2 or max_x_1<min_x_2 or min_y_1>max_y_2 or max_y_1<min_y_2:
#             check_collision = False
#         if check_collision:
#             break
#     return check_collision            
        
    
def execute(self, inputs, outputs, gvm):
    self.logger.info("Hello {}".format(self.name))
    planning_env_msg = get_planning_environment()
    planning_env = PlanningEnvironmentRos.from_ros_msg(planning_env_msg)
    row = gvm.get_variable("row", per_reference=False, default=None)
    #生成的所有面的物料ID位置信息
    all_pose_dict = gvm.get_variable("all_pose_dict", per_reference=False, default=None)
    
    #当前拣配托盘朝向
    current_direction = inputs["current_direction"]

    check_pose_dict = all_pose_dict[current_direction]

    #当前拣配托盘料箱ID
    pallet_tote_data = inputs["pallet_tote_data"]
    new_pose_dict = {}

    #更新拣配托盘位置ID信息
    for box_id,value in pallet_tote_data.items():
        new_pose_dict[int(box_id)] = check_pose_dict[int(box_id)]    
        
    #获取拣配托盘所有箱子
    pick_container_items = planning_env.get_container_items("0")

    #获取拣配托盘坐标
    pick_workspace_ros = planning_env.get_workspace_ros("0")
    tf_base_pick_space = pick_workspace_ros.get_bottom_pose()
    
    if len(pallet_tote_data)!=len(pick_container_items):
        self.logger.info("拣配托盘上物料数量和视觉上物料数量不一致")
        raise "拣配托盘上物料数量和视觉上物料数量不一致"

    #匹配当前箱子条码     
    for container_item in pick_container_items:
        tf_base_box_real = SE3(pose_to_list(container_item.origin))
        tf_space_box_real = (tf_base_pick_space.inv())*tf_base_box_real
        space_box_pose_real = tf_space_box_real.xyz_quat
        #self.logger.info(f"拣配视觉料箱坐标XYZ坐标为 {space_box_pose_real[0:3]}")
        for key,values in new_pose_dict.items():    
            check_list = list(filter(lambda x:abs(space_box_pose_real[x]-values[x])<0.09,[0,1]))    
            check_list_z = list(filter(lambda x:abs(space_box_pose_real[x]-values[x])<0.05,[2]))        
            if len(check_list)==2 and check_list_z:
                box_id = key
                break
            else:
                box_id = -1       
        if box_id<0:
            self.logger.error("未能匹配上位置号")
            raise "未能匹配上位置号,可能是实际托盘朝向与托盘朝向不一致"
        else:    
            self.logger.info(f"匹配到位置id为{box_id}")    
        #添加box_id
        box_id = str(box_id) 
        #添加信息到环境
        if box_id in pallet_tote_data.keys():
            #更新料箱ID到环境
            if "box_id" in container_item.additional_info.keys:
                index = container_item.additional_info.keys.index("box_id")
                container_item.additional_info.values[index] = box_id
            else:
                container_item.additional_info.keys.append("box_id")   
                container_item.additional_info.values.append(box_id)   
            #更新条码到环境       
            if "barcode" in container_item.additional_info.keys:
                index = container_item.additional_info.keys.index("barcode")
                container_item.additional_info.values[index] = pallet_tote_data[box_id]["barcode"]     
            else:    
                container_item.additional_info.keys.append("barcode")
                container_item.additional_info.values.append(pallet_tote_data[box_id]["barcode"]) 
            #更新to_ws到环境,主要是因为一致性,实际用处不大       
            if "to_ws" in container_item.additional_info.keys:
                index = container_item.additional_info.keys.index("to_ws")
                container_item.additional_info.values[index] = ""    
            else:    
                container_item.additional_info.keys.append("to_ws")
                container_item.additional_info.values.append("")  
                 
                    
            if row==5:
                row_id = int(box_id)%row
                lay_id = int(box_id)//row                  
                if row_id in [1,2,3]:                    
                    tf_base_box_real = SE3([0.002,-0.002,0,0,0,0,1])*tf_base_box_real
                elif row_id in [0,4]:
                    tf_base_box_real = tf_base_box_real*SE3([0.00,0.00,0,0,0,0,1])     
            elif row==9:
                row_id = int(box_id)%row
                lay_id = int(box_id)//row
                if row_id in [1,2]:
                    tf_base_box_real = tf_base_box_real*SE3([0.00,0.001,0,0,0,0,1]) 
                elif row_id in [3,4]:     
                    tf_base_box_real = tf_base_box_real*SE3([0.00,0.001,0,0,0,0,1]) 
                elif row_id in [5]:   
                    if lay_id<2:
                        tf_base_box_real = tf_base_box_real*SE3([-0.004,0.0035,0,0,0,0,1])  
                    else:
                        tf_base_box_real = tf_base_box_real*SE3([-0.004,0.0035,0,0,0,0,1])                       
                elif row_id in [6]:  
                    if lay_id<2:
                        tf_base_box_real = tf_base_box_real*SE3([-0.001,0.0015,0,0,0,0,1]) 
                    else:
                        tf_base_box_real = tf_base_box_real*SE3([-0.001,0.0015,0,0,0,0,1])                                                                
                elif row_id in [7]:
                    if lay_id<2:
                        tf_base_box_real = tf_base_box_real*SE3([0.00,0.001,0,0,0,0,1]) 
                    else:
                        tf_base_box_real = tf_base_box_real*SE3([0.00,0.001,0,0,0,0,1])                      
                elif row_id in [8]:
                    tf_base_box_real = tf_base_box_real*SE3([0.00,-0.005,0,0,0,0,1])                       
                elif row_id in [0]:
                    if lay_id<2:
                        tf_base_box_real = tf_base_box_real*SE3([0.00,0.00,0,0,0,0,1])  
                    else:
                        tf_base_box_real = tf_base_box_real*SE3([0.00,0.00,0,0,0,0,1])                            
            else:
                raise "无效的row"   
                     
            container_item.origin = Pose(*tf_base_box_real.xyz_quat)                   
        else:
            self.logger.info(f"托盘数据缺少位置号{box_id}")
            raise f"托盘数据缺少位置号"               
    
    #获取拣配托盘上放置规划箱子
    pick_plan_items = planning_env.get_unfinished_planned_items("0")  
    #获取所有朝向的放置规划,并根据当前朝向更新
    all_direction_plan_items = all_direction_items(planning_env,pick_plan_items,"C")
    pick_plan_items = all_direction_plan_items[current_direction]["container_items"]
        
    
    check_plan_list = []
    for container_item in pick_container_items:
        tf_box_real = SE3(pose_to_list(container_item.origin))
        #通过视觉的箱子找到放置规划的箱子
        remove_plan_item = list(filter(lambda x:np.allclose(pose_to_list(x.origin)[0:2],tf_box_real.xyz_quat[0:2],atol=0.085)\
            and np.allclose(pose_to_list(x.origin)[2],tf_box_real.xyz_quat[2],atol=0.05),pick_plan_items))
        if not remove_plan_item:
            self.logger.info(f"拣配托盘上没有找到匹配的箱子{container_item.name}")
            raise f"拣配托盘上没有找到匹配的箱子"
        remove_plan_item = remove_plan_item[0]    
        #移除放置规划的箱子
        pick_plan_items.remove(remove_plan_item)
        check_plan_list.append(remove_plan_item)
    
    if len(check_plan_list)!=len(pick_container_items):
        self.logger.info(f"拣配托盘放置规划匹配到{len(check_plan_list)}个箱子,实际视觉有{len(pick_container_items)}个箱子")   
        raise f"拣配托盘放置规划匹配箱子和实际视觉箱子数量不匹配"

    # new_pick_plan_items = []    
    #将位置ID赋值到放置规划    
    for plan_item in pick_plan_items:
        tf_space_box_real = (tf_base_pick_space.inv())*SE3(pose_to_list(plan_item.origin))
        space_box_pose_real = tf_space_box_real.xyz_quat
        self.logger.info(f"放置规划XYZ坐标为 {space_box_pose_real[0:3]}")
        for key,values in check_pose_dict.items():    
            check_list = list(filter(lambda x:abs(space_box_pose_real[x]-values[x])<0.085,[0,1]))   
            check_list_z = list(filter(lambda x:abs(space_box_pose_real[x]-values[x])<0.05,[2]))           
            if len(check_list)==2 and check_list_z:
                box_id = key
                break
            else:
                box_id = -1       
        if box_id<0:
            self.logger.error("未能匹配上位置号")
            raise "未能匹配上位置号,比较奇怪的bug"
        else:    
            self.logger.info(f"匹配到位置id为{box_id}") 
        box_id = str(box_id)
        #更新料箱ID到环境
        if "box_id" in plan_item.additional_info.keys:
            index = plan_item.additional_info.keys.index("box_id")
            plan_item.additional_info.values[index] = box_id
        else:
            plan_item.additional_info.keys.append("box_id")   
            plan_item.additional_info.values.append(box_id)   
        #更新条码到环境       
        if "barcode" in plan_item.additional_info.keys:
            index = plan_item.additional_info.keys.index("barcode")
            plan_item.additional_info.values[index] = ""    
        else:    
            plan_item.additional_info.keys.append("barcode")
            plan_item.additional_info.values.append("") 
        #更新to_ws到环境,主要是因为一致性,实际用处不大       
        if "to_ws" in plan_item.additional_info.keys:
            index = plan_item.additional_info.keys.index("to_ws")
            plan_item.additional_info.values[index] = ""    
        else:    
            plan_item.additional_info.keys.append("to_ws")
            plan_item.additional_info.values.append("")   
        # new_pick_plan_items.append(plan_item) 
    if not pick_container_items:
        clear_container_all_items("0")
        add_container_items("0",pick_container_items)
        set_planned_items("0",pick_plan_items,[])
        return "no_container_items"
    #通过记录的拣配任务抓取路径过滤planitems
    path = inputs["path"]
    history_path = []
    for key,item in path.items():
        if isinstance(item, dict):
            if "pick_path" in item.keys():
                history_path+=item["pick_path"]            
    self.logger.info(f"拣配任务已抓取路径为{history_path}")   
        
    pick_plan_items = list(filter(lambda item: item.additional_info.values[-3] in history_path, pick_plan_items))
    

    #耦合抓取箱子和放置规划的箱子
    bottom_plan_items = filter_bottom_items(pick_plan_items)

    bottom_container_items = filter_bottom_items(pick_container_items)
    init_no_overlap_plan_items = []
    #row = gvm.get_variable("row", per_reference=False, default=None)
    if len(bottom_container_items)==row or not bottom_container_items:
        update_plan_items = []
        no_overlap_plan_items = []
        self.logger.info(f"已有箱子已填满底部，无需偏移放置箱子")
    else:    
        #找到与视觉未重合的那一列放置规划
        for bottom_plan_item in bottom_plan_items:
            for bottom_container_item in bottom_container_items:
                check_list = list(filter(lambda x:abs(pose_to_list(bottom_plan_item.origin)[x]-pose_to_list(bottom_container_item.origin)[x])<0.085,[0,1]))  
                if len(check_list)==2:
                    break
            if not len(check_list)==2:
                init_no_overlap_plan_items.append(bottom_plan_item)     
                                              
        fuc_check_collision = lambda check_plan_item: is_collision(check_plan_item,bottom_container_items)
        
        #拿到原先抓取箱子的位置 ----只有底层箱子
        from_pick_pose_dict = {}
        for key,item in path["from_pick_pose_dict"].items():
            if key in [str(i) for i in range(1,row+1)]:
                from_pick_pose_dict[key] = item   
                
        #让空的那列箱子还原回原先的位置
        for item in init_no_overlap_plan_items:
            item_id = item.additional_info.values[-3]
            if item_id in from_pick_pose_dict.keys():
                item.origin = Pose(*from_pick_pose_dict[item_id])
                
        #得到初始干涉的料箱
        init_collision_items = list(filter(fuc_check_collision,init_no_overlap_plan_items))
        init_collision_items_id = [i.additional_info.values[-3] for i in init_collision_items]          
        #如果存在初始干涉的料箱，则进行偏移                    
        if init_collision_items:
            self.logger.info(f"存在放置规划的箱子与已有箱子干涉 {init_collision_items_id}，偏移中")
            #拆分xy偏移精度
            def split_slide(slide,precision):
                split_x_length = int(slide[0]/precision)
                split_y_length = int(slide[1]/precision)
                new_slide_x = []
                new_slide_y = []
                for i in range(1,abs(split_x_length)+1):
                    if split_x_length>0:
                        new_slide_x.append(i*precision)
                    else:
                        new_slide_x.append(-i*precision)        
                for i in range(1,abs(split_y_length)+1):
                    if split_y_length>0:
                        new_slide_y.append(i*precision)
                    else:
                        new_slide_y.append(-i*precision)   
                return_list = list(product(new_slide_y,new_slide_y))        
                #return_list = [[x,y] for x,y in zip(new_slide_x,new_slide_y)]                               
                return return_list   
            #添加偏移点
            slide_list = split_slide([0.085,0.085],0.005)  
            slide_list+= split_slide([0.085,-0.085],0.005) 
            slide_list+= split_slide([-0.085,0.085],0.005) 
            slide_list+= split_slide([-0.085,-0.085],0.005) 
            slide_list = sorted(slide_list,key=lambda x:abs(x[0])+abs(x[1]))   
              
            #生成所有碰撞箱子排列组合
            all_collision_items_tupple = list(permutations(init_no_overlap_plan_items))
            all_collision_items_list = [list(x) for x in all_collision_items_tupple]
            
            #排序，优先处理与初始干涉的箱子相同的箱子
            all_collision_items_list = \
            sorted(all_collision_items_list,key=lambda x:tuple([x[0].additional_info.values[-3]==i for i in init_collision_items_id]),reverse=True)
            if self.smart_data["multiprocess"]:                       
                #计算结果进程队列   
                calculation_queue = multiprocessing.Queue()
                #终止事件   
                termination_event = multiprocessing.Event()  
                bottom_container_items_msg = dill.dumps(bottom_container_items)
                calculation_process_list = []
                calculation_flag = False
                                  
                #多进程处理
                for collision_item_id in init_collision_items_id:
                    calcu_palletize_async = CALCU_PALLETIZE_ASYNC()
                    #添加进程消息队列
                    calcu_palletize_async.calculation_queue = calculation_queue
                    #添加进程终止事件
                    calcu_palletize_async.termination_event = termination_event
                    calcu_palletize_async.logger = self.logger.info
                    #启动计算进程
                    #根据碰撞的箱子,生成新的排列组合,过滤掉原先的排列组合
                    append_collision_items_list = list(filter(lambda x:x[0].additional_info.values[-3]==collision_item_id,all_collision_items_list))
                    append_collision_items_list_msg = dill.dumps(append_collision_items_list)
                    #添加进程输入参数
                    async_input = (slide_list,append_collision_items_list_msg,bottom_container_items_msg,from_pick_pose_dict)
                    calculation_process = multiprocessing.Process(target=calcu_palletize_async.get_update_items, args=async_input)
                    calculation_process.start()
                    calculation_process_list.append(calculation_process)
                #剩余非碰撞的排列组合    
                append_collision_items_list = list(filter(lambda x:x[0].additional_info.values[-3] not in init_collision_items_id,all_collision_items_list))
                append_collision_items_list_msg = dill.dumps(append_collision_items_list)
                #添加进程输入参数
                async_input = (slide_list,append_collision_items_list_msg,bottom_container_items_msg,from_pick_pose_dict)
                calculation_process = multiprocessing.Process(target=calcu_palletize_async.get_update_items, args=async_input)
                calculation_process.start()
                calculation_process_list.append(calculation_process)                
                        
                if not calculation_flag:      
                    try:       
                        #获取进程队列结果   
                        no_overlap_plan_items,update_plan_items = calculation_queue.get(timeout=60)
                    except:
                        self.logger.info("计算超时")
                        #设置终止事件，让其他进程计算结束
                        termination_event.set() 
                        self.logger.info("calculation time out")
                        for process in calculation_process_list:
                            try:
                                process.join(timeout=6) 
                                if process.is_alive():
                                    self.logger.info(f"{process.pid}进程卡死") 
                                    process.kill() 
                            except Exception as e:
                                self.logger.info(f"{process.pid}进程异常") 
                                process.kill()
                        time.sleep(2)              
                        calculation_queue.close()
                        termination_event.clear()
                        raise "计算超时"   
                #设置终止事件，让其他进程计算结束
                termination_event.set() 
                #等待其他进程计算结束
                self.logger.info("calculation_complete")
                termination_event.wait() 
                
                for process in calculation_process_list:
                    try:
                        process.join(timeout=1) 
                        if process.is_alive():
                            self.logger.info(f"{process.pid}进程卡死") 
                            process.kill() 
                    except Exception as e:
                        self.logger.info(f"{process.pid}进程异常")  
                        process.kill()  
                    
            else:                 
                break_flag = False
                #排列组合依次做obb碰撞检测偏移
                for index,collision_items_list in enumerate(all_collision_items_list):              
                    if break_flag == True:
                        break
                    stack = Stack()
                    no_overlap_plan_items = copy.deepcopy(collision_items_list)
                    #删除掉干涉的箱子
                    no_overlap_plan_items.remove(collision_items_list[0])
                    #添加干涉的箱子，已偏移的箱子到栈数据
                    stack.push([collision_items_list[0],[],])
                    while not stack.is_empty(): #and not self.preempted:
                        #调出干涉的箱子，已偏移的箱子的栈数据
                        init_collision_item, update_plan_items = stack.pop() 
                        #计算更新碰撞 
                        collision_flag = True
                        
                        #拿到箱子在拣配托盘的ID
                        init_collision_item_id = init_collision_item.additional_info.values[-3]
                        #判断是否使用原先箱子的位置
                        if init_collision_item_id in from_pick_pose_dict.keys():
                            from_pick_pose = from_pick_pose_dict[init_collision_item_id]
                            self.logger.info(f"应用原先箱子{init_collision_item_id}的坐标,开始计算")
                            #x,y偏移的量
                            x_length = init_collision_item.origin.x-from_pick_pose[0]
                            y_length = init_collision_item.origin.y-from_pick_pose[1] 
                            #添加偏移点
                            new_slide_list = split_slide([0.085-x_length,0.085-y_length],0.005)  
                            new_slide_list+= split_slide([0.085-x_length,-0.085-y_length],0.005) 
                            new_slide_list+= split_slide([-0.085-x_length,0.085-y_length],0.005) 
                            new_slide_list+= split_slide([-0.085-x_length,-0.085-y_length],0.005) 
                            new_slide_list = sorted(new_slide_list,key=lambda x:abs(x[0])+abs(x[1]))
                            old_collision_item = copy.deepcopy(init_collision_item)                       
                            old_collision_item.origin = Pose(*from_pick_pose)
                            #拿到箱子旋转量
                            tf_box_rotate = get_rotate_angle(init_collision_item,bottom_container_items+update_plan_items).xyz_quat                   
                            #再对箱子进行平移
                            for slide in new_slide_list:
                                if self.termination_event.is_set():
                                    return False
                                collision_item = copy.deepcopy(old_collision_item)
                                collision_item.origin.x+=slide[0]
                                collision_item.origin.y+=slide[1]
                                if not is_collision(collision_item,update_plan_items+bottom_container_items):
                                    collision_flag = False
                                    break
                            #如果干涉则采用旋转+初始规划箱子偏置    
                            if collision_flag:  
                                self.logger.info(f"原先箱子{init_collision_item_id}计算失败,开始计算旋转+初始规划箱子偏置")                        
                                #拿到箱子旋转量
                                tf_box_rotate = get_rotate_angle(init_collision_item,bottom_container_items+update_plan_items).xyz_quat                   
                                #再对箱子进行平移
                                for slide in slide_list:
                                    if self.termination_event.is_set():
                                        return False
                                    #先对箱子进行旋转
                                    collision_item = copy.deepcopy(init_collision_item)
                                    new_tf_base_box = SE3(pose_to_list(init_collision_item.origin))*SE3(tf_box_rotate)
                                    collision_item.origin = Pose(*new_tf_base_box.xyz_quat)
                                    collision_item.origin.x+=slide[0]
                                    collision_item.origin.y+=slide[1]
                                    if not is_collision(collision_item,update_plan_items+bottom_container_items):
                                        collision_flag = False
                                        break   
                            else:
                                self.logger.info(f"使用原先箱子{init_collision_item_id}坐标计算成功")
                        #无原先拣配箱子的续码规划              
                        else:                                                              
                            #拿到箱子旋转量
                            tf_box_rotate = get_rotate_angle(init_collision_item,bottom_container_items+update_plan_items).xyz_quat
                            
                            #再对箱子进行平移
                            for slide in slide_list:
                                #先对箱子进行旋转
                                collision_item = copy.deepcopy(init_collision_item)
                                new_tf_base_box = SE3(pose_to_list(init_collision_item.origin))*SE3(tf_box_rotate)
                                collision_item.origin = Pose(*new_tf_base_box.xyz_quat)
                                collision_item.origin.x+=slide[0]
                                collision_item.origin.y+=slide[1]
                                if not is_collision(collision_item,update_plan_items+bottom_container_items):
                                    collision_flag = False
                                    break
                            
                        if not collision_flag:
                            update_plan_items += [collision_item]
                            fuc_check_collision = lambda check_plan_item: is_collision(check_plan_item,update_plan_items+bottom_container_items)
                            #得到干涉的箱子
                            collision_items = list(filter(fuc_check_collision,no_overlap_plan_items))
                            #import ipdb;ipdb.set_trace()
                            if not collision_items:
                                break_flag = True
                                break
                            else:
                                no_overlap_plan_items.remove(collision_items[0])
                                stack.push([collision_items[0],update_plan_items,])
                        else:
                            if index+1==len(all_collision_items_list):
                                self.logger.info(f"不管如何偏移都会计算失败")
                                raise "不管如何偏移都会计算失败"
                            else:
                                self.logger.info(f"不管如何偏移都会计算失败,使用下一组解计算")
                                break            
        else:
            update_plan_items = []
            no_overlap_plan_items = copy.deepcopy(init_no_overlap_plan_items)
                    
    update_plan_items = no_overlap_plan_items+update_plan_items  
                  
    for slot in pick_plan_items:
        #判断是否和实际箱子xy重合
        for item in pick_container_items: 
            tf_box_real = pose_to_list(item.origin)
            tf_plan_box = pose_to_list(slot.origin)
            check_list = list(filter(lambda x:abs(tf_box_real[x]-tf_plan_box[x])<0.085,[0,1]))  
            if len(check_list)==2:
                break
        #与已有箱子未重合则判断是否那一列为空，并且更新    
        if not len(check_list)==2:
            for item in update_plan_items:
                tf_update_plan = pose_to_list(item.origin)   
                check_list = list(filter(lambda x:abs(tf_update_plan[x]-tf_plan_box[x])<0.085,[0,1]))   
                if len(check_list)==2:
                    break  
            if not len(check_list)==2:                              
                self.logger.info(f"视觉x,y未能匹配到 plan item")
                raise "视觉x,y未能匹配到plan item"  
            else:
                tf_update_box = tf_update_plan 
                tf_update_box[2] = slot.origin.z
                slot.origin = Pose(*tf_update_box)                
        #与已有箱子重合则更新xy坐标        
        else:        
            tf_update_box = tf_box_real 
            tf_update_box[2] = slot.origin.z
            slot.origin = Pose(*tf_update_box)
    #import ipdb;ipdb.set_trace()    
    
                    
    clear_container_all_items("0")
    add_container_items("0",pick_container_items)
    set_planned_items("0",pick_plan_items,[])
    return "success"
