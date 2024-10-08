import copy
from xyz_motion import SE3
import tf.transformations as tfm
import numpy as np
from xyz_env_manager.msg import Pose
import os 
import dill

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

def pose_to_list(container_item_origin):
   tf_map_object = []
   tf_map_object.append(container_item_origin.x)
   tf_map_object.append(container_item_origin.y)
   tf_map_object.append(container_item_origin.z)
   tf_map_object.append(container_item_origin.qx)
   tf_map_object.append(container_item_origin.qy)
   tf_map_object.append(container_item_origin.qz)
   tf_map_object.append(container_item_origin.qw)   
   return tf_map_object


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
    return_list = [[x,y] for x,y in zip(new_slide_x,new_slide_y)]                               
    return return_list   

class CALCU_PALLETIZE_ASYNC():

    def __init__(self) -> None:
      # 进程计算队列
      self.calculation_queue = None
      # 进程终止事件
      self.termination_event = None
      # 日志
      self.logger = None
       
    def get_update_items(self,slide_list,append_collision_items_list_msg,bottom_container_items_msg,from_pick_pose_dict):
        try:
            append_collision_items_list = dill.loads(append_collision_items_list_msg)
            bottom_container_items = dill.loads(bottom_container_items_msg)
         
            # 获取当前进程的 ID
            current_process_id = os.getpid()
            self.logger(f"{current_process_id}进程,进程终止事件状态{self.termination_event.is_set()}")
            break_flag = False
            #排列组合依次做obb碰撞检测偏移
            for index,collision_items_list in enumerate(append_collision_items_list):
                collision_items_id_list = [i.additional_info.values[-3] for i in collision_items_list]
                if break_flag == True:
                    break
                stack = Stack()
                no_overlap_plan_items = copy.deepcopy(collision_items_list)
                #删除掉干涉的箱子
                no_overlap_plan_items.remove(collision_items_list[0])
                #添加干涉的箱子，已偏移的箱子到栈数据
                stack.push([collision_items_list[0],[],])
                while not stack.is_empty() and not self.termination_event.is_set(): #and not self.preempted:
                    #调出干涉的箱子，已偏移的箱子的栈数据
                    init_collision_item, update_plan_items = stack.pop() 
                    #计算更新碰撞 
                    collision_flag = True
                    #拿到箱子在拣配托盘的ID
                    init_collision_item_id = init_collision_item.additional_info.values[-3]
                    #判断是否使用原先箱子的位置
                    if init_collision_item_id in from_pick_pose_dict.keys():
                        from_pick_pose = from_pick_pose_dict[init_collision_item_id]
                        self.logger(f"{current_process_id}进程,偏置列表{collision_items_id_list},应用原先箱子{init_collision_item_id}的坐标,开始计算")
                        #x,y偏移的量
                        x_length = init_collision_item.origin.x-from_pick_pose[0]
                        y_length = init_collision_item.origin.y-from_pick_pose[1]
                                
                        #添加偏移点
                        new_slide_list = split_slide([0.08-x_length,0.08-y_length],0.005)  
                        new_slide_list+= split_slide([0.08-x_length,-0.08-y_length],0.005) 
                        new_slide_list+= split_slide([-0.08-x_length,0.08-y_length],0.005) 
                        new_slide_list+= split_slide([-0.08-x_length,-0.08-y_length],0.005) 
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
                            self.logger(f"{current_process_id}进程,偏置列表{collision_items_id_list},原先箱子{init_collision_item_id}计算失败,开始计算旋转+初始规划箱子偏置")                   
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
                            self.logger(f"{current_process_id}进程,偏置列表{collision_items_id_list},使用原先箱子{init_collision_item_id}坐标计算成功")                                     
                    #无原先拣配箱子的续码规划        
                    else:
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
                                                 
                    if not collision_flag:
                        update_plan_items += [collision_item]
                        fuc_check_collision = lambda check_plan_item: is_collision(check_plan_item,update_plan_items+bottom_container_items)
                        #得到干涉的箱子
                        collision_items = list(filter(fuc_check_collision,no_overlap_plan_items))
                        #import ipdb;ipdb.set_trace()
                        if not collision_items:
                            break_flag = True
                            self.calculation_queue.put((no_overlap_plan_items,update_plan_items))
                            self.logger(f"进程ID{current_process_id},{collision_items_id_list}计算成功")
                            break
                        else:
                            no_overlap_plan_items.remove(collision_items[0])
                            stack.push([collision_items[0],update_plan_items,])
                    else:
                        if index+1==len(append_collision_items_list):
                            self.logger(f"进程ID{current_process_id},不管如何偏移都会计算失败")
                        else:
                            self.logger(f"进程ID{current_process_id},{collision_items_id_list}不管如何偏移都会计算失败,使用下一组解计算")
                            break    
        except Exception as e:
            self.logger(f"{current_process_id}进程,计算异常,异常信息{str(e)}")          