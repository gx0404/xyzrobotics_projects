from xyz_motion import PlanningEnvironmentRos,pose_to_list,SE3
from xyz_env_manager.client import get_planning_environment
import numpy as np
from xyz_env_manager.client import set_planned_items,clear_planned_items
from xyz_env_manager.client import clear_container_all_items,add_container_items
import tf.transformations as tfm
import copy
from xyz_env_manager.msg import Pose

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

def get_rect_corners(item):
    l,w,h = item.primitives[0].dimensions
    tfs_box_vertices = [[l/2, w/2, 0, 0, 0, 0, 1], [-l/2, w/2, 0, 0, 0, 0, 1], [-l/2, -w/2, 0, 0, 0, 0, 1], [l/2, -w/2, 0, 0, 0, 0, 1]]
    tf_world_box = SE3(pose_to_list(item.origin))
    tfs_world_vertices = [(tf_world_box * SE3(tf)).xyz_quat for tf in tfs_box_vertices]
    max_x = max([tf[0] for tf in tfs_world_vertices])
    min_x = min([tf[0] for tf in tfs_world_vertices])
    max_y = max([tf[1] for tf in tfs_world_vertices])
    min_y = min([tf[1] for tf in tfs_world_vertices])
    return max_x,min_x,max_y,min_y

def is_collision(check_plan_item,other_plan_items):
    max_x_1,min_x_1,max_y_1,min_y_1 = get_rect_corners(check_plan_item)
    for plan_item in other_plan_items:
        check_collision = True
        max_x_2,min_x_2,max_y_2,min_y_2 = get_rect_corners(plan_item)
        if min_x_1>max_x_2 or max_x_1<min_x_2 or min_y_1>max_y_2 or max_y_1<min_y_2:
            check_collision = False
        if check_collision:
            break
    return check_collision            
        
    
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
        self.logger.info(f"拣配视觉料箱坐标XYZ坐标为 {space_box_pose_real[0:3]}")
        for key,values in new_pose_dict.items():    
            check_list = list(filter(lambda x:abs(space_box_pose_real[x]-values[x])<0.08,[0,1]))    
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
                    tf_base_box_real = tf_base_box_real*SE3([0.002,0.002,0,0,0,0,1]) 
                elif row_id in [3,4]:     
                    tf_base_box_real = tf_base_box_real*SE3([0.0015,0.0015,0,0,0,0,1]) 
                elif row_id in [5,6]: 
                    tf_base_box_real = tf_base_box_real*SE3([-0.002,0.00,0,0,0,0,1])                           
                elif row_id in [7]:
                    tf_base_box_real = tf_base_box_real*SE3([0.001,-0.003,0,0,0,0,1])   
                elif row_id in [8]:
                    tf_base_box_real = tf_base_box_real*SE3([0.001,0.001,0,0,0,0,1])                       
                elif row_id in [0]:
                    tf_base_box_real = tf_base_box_real*SE3([0.001,-0.00,0,0,0,0,1])                    
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
        remove_plan_item = list(filter(lambda x:np.allclose(pose_to_list(x.origin)[0:2],tf_box_real.xyz_quat[0:2],atol=0.08)\
            and np.allclose(pose_to_list(x.origin)[2],tf_box_real.xyz_quat[2],atol=0.05),pick_plan_items))
        if not remove_plan_item:
            self.logger.info(f"拣配托盘上没有找到匹配的箱子{container_item.name}")
            raise f"拣配托盘上没有找到匹配的箱子"
        remove_plan_item = remove_plan_item[0]    
        #移除放置规划的箱子
        pick_plan_items.remove(remove_plan_item)
        check_plan_list.append(remove_plan_item)


    #耦合抓取箱子和放置规划的箱子
    bottom_plan_items = filter_bottom_items(pick_plan_items)

    bottom_container_items = filter_bottom_items(pick_container_items)
    no_overlap_plan_items = []
    if len(bottom_container_items)==9:
        update_plan_items = []
        self.logger.info(f"已有箱子已填满底部，无需偏移放置箱子")
    else:    
        #找到与视觉未重合的那一列放置规划
        for bottom_plan_item in bottom_plan_items:
            for bottom_container_item in bottom_container_items:
                check_list = list(filter(lambda x:abs(pose_to_list(bottom_plan_item.origin)[x]-pose_to_list(bottom_container_item.origin)[x])<0.08,[0,1]))  
                if len(check_list)==2:
                    break
            if not len(check_list)==2:
                no_overlap_plan_items.append(bottom_plan_item)     
                                            
        # stack = Stack()  
        # fuc_check_collision = lambda check_plan_item: is_collision(check_plan_item,bottom_container_items)
        # #得到干涉的箱子
        # collision_items = list(filter(fuc_check_collision,no_overlap_plan_items))
        # if collision_items:
        #     self.logger.info(f"存在放置规划的箱子与已有箱子干涉，偏移中")
        #     #添加偏移点
        #     slide_list = [(x/100, y/100) for x in range(1, 7) for y in range(1, 7)]  
        #     slide_list+= [(-x/100, y/100) for x in range(1, 7) for y in range(1, 7)] 
        #     slide_list+= [(x/100, -y/100) for x in range(1, 7) for y in range(1, 7)] 
        #     slide_list+= [(-x/100, -y/100) for x in range(1, 7) for y in range(1, 7)] 
        #     slide_list = sorted(slide_list,key=lambda x:abs(x[0])+abs(x[1]))
        #     #删除掉干涉的箱子
        #     no_overlap_plan_items.remove(collision_items[0])
        #     #添加干涉的箱子，已偏移的箱子到栈数据
        #     stack.push([collision_items[0],[],])
        #     while not stack.is_empty() and not self.preempted:
        #         #调出干涉的箱子，已偏移的箱子的栈数据
        #         init_collision_item, update_plan_items = stack.pop() 
        #         #计算更新碰撞 
        #         collision_flag = True
        #         for slide in slide_list:
        #             collision_item = copy.deepcopy(init_collision_item)
        #             collision_item.origin.x += slide[0]
        #             collision_item.origin.y += slide[1]
        #             if not is_collision(collision_item,update_plan_items+bottom_container_items):
        #                 collision_flag = False
        #                 break
                    
        #         if not collision_flag:
        #             update_plan_items += [collision_item]
        #             fuc_check_collision = lambda check_plan_item: is_collision(check_plan_item,update_plan_items+bottom_container_items)
        #             #得到干涉的箱子
        #             collision_items = list(filter(fuc_check_collision,no_overlap_plan_items))
        #             if not collision_items:
        #                 break
        #             else:
        #                 no_overlap_plan_items.remove(collision_items[0])
        #                 stack.push([collision_items[0],update_plan_items,])
        #         else:
        #             self.logger.info(f"不管如何偏移都会计算失败")
        #             raise "不管如何偏移都会计算失败"             
        # else:
        #     update_plan_items = []    
    update_plan_items = []              
    update_plan_items = no_overlap_plan_items+update_plan_items                
    for slot in pick_plan_items:
        #判断是否和实际箱子xy重合
        for item in pick_container_items: 
            tf_box_real = pose_to_list(item.origin)
            tf_plan_box = pose_to_list(slot.origin)
            check_list = list(filter(lambda x:abs(tf_box_real[x]-tf_plan_box[x])<0.08,[0,1]))  
            if len(check_list)==2:
                break
        #与已有箱子未重合则判断是否那一列为空，并且更新    
        if not len(check_list)==2:
            for item in update_plan_items:
                tf_update_plan = pose_to_list(item.origin)   
                check_list = list(filter(lambda x:abs(tf_update_plan[x]-tf_plan_box[x])<0.08,[0,1]))   
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
    new_pick_plan_items = []    
    #将位置ID赋值到放置规划    
    for plan_item in pick_plan_items:
        tf_space_box_real = (tf_base_pick_space.inv())*SE3(pose_to_list(plan_item.origin))
        space_box_pose_real = tf_space_box_real.xyz_quat
        self.logger.info(f"放置规划XYZ坐标为 {space_box_pose_real[0:3]}")
        for key,values in check_pose_dict.items():    
            check_list = list(filter(lambda x:abs(space_box_pose_real[x]-values[x])<0.08,[0,1]))   
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
        new_pick_plan_items.append(plan_item)
    if len(check_plan_list)!=len(pick_container_items):
        self.logger.info(f"拣配托盘放置规划匹配到{len(check_plan_list)}个箱子,实际视觉有{len(pick_container_items)}个箱子")   
        raise f"拣配托盘放置规划匹配箱子和实际视觉箱子数量不匹配"
    
                    
    clear_container_all_items("0")
    add_container_items("0",pick_container_items)
    set_planned_items("0",new_pick_plan_items,[])
    return "success"
