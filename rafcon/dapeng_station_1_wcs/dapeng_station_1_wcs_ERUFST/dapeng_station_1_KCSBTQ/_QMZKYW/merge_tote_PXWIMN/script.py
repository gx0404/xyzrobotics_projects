import numpy as np
from xyz_motion import SE3,pose_to_list
import tf.transformations as tfm
from xyz_env_manager.msg import Pose
import copy
from xyz_motion import PlanningEnvironmentRos
from xyz_env_manager.client import get_planning_environment, set_planned_items, clear_container_all_items

def merge_items(slot_items):
    min_x = 1e9
    min_y = 1e9
    max_x = -1e9
    max_y = -1e9
    h_list = []
    z_list = []
    box_vertices = np.array([[0, 0]])
    for item in slot_items:
        l = item.primitives[0].dimensions[0]
        w = item.primitives[0].dimensions[1]
        h = item.primitives[0].dimensions[2]
        tfs_box_vertices = [[l/2, w/2, 0, 0, 0, 0, 1], [-l/2, w/2, 0, 0, 0, 0, 1], [-l/2, -w/2, 0, 0, 0, 0, 1], [l/2, -w/2, 0, 0, 0, 0, 1]]
        tf_world_box = SE3(pose_to_list(item.origin))
        tfs_world_vertices = [(tf_world_box * SE3(tf)).xyz_quat for tf in tfs_box_vertices]
        vertices_x = [tf[0] for tf in tfs_world_vertices]
        vertices_y = [tf[1] for tf in tfs_world_vertices]
        for i in range(len(vertices_x)):
            box_vertices = np.concatenate((box_vertices, np.array([[vertices_x[i], vertices_y[i]]])), axis = 0)
        z_list.append(pose_to_list(item.origin)[2])
        h_list.append(h)    

    #box_vertices = np.array([[max_x, max_y], [min_x, max_y], [min_x, min_y], [max_x, min_y]])
    box_vertices = np.delete(box_vertices, 0, 0)
    box_vertices = np.array(box_vertices, dtype = np.float32)
    import cv2
    import math
    min_rot_rect = cv2.minAreaRect(box_vertices)
    new_xyz = [min_rot_rect[0][0], min_rot_rect[0][1], sum(z_list)/len(z_list)]
    if min_rot_rect[1][0] < min_rot_rect[1][1]:
        new_quat = list(tfm.quaternion_from_euler(0, 0, (min_rot_rect[2]-90)/180*math.pi))
        new_length = min_rot_rect[1][1]
        new_width = min_rot_rect[1][0]
        new_height = sum(h_list)/len(h_list)
    else:
        new_quat = list(tfm.quaternion_from_euler(0, 0, min_rot_rect[2]/180*math.pi))
        new_length = min_rot_rect[1][0]
        new_width = min_rot_rect[1][1]
        new_height = sum(h_list)/len(h_list)
    
    new_item = copy.deepcopy(slot_items[0])
    new_pose = new_xyz + new_quat
    new_item.origin = Pose(*new_pose)         
    new_item.primitives[0].dimensions = (new_length,new_width,new_height)
    return new_item

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

def execute(self, inputs, outputs, gvm):
    self.logger.info("Hello {}".format(self.name))
    place_id = self.smart_data["place_workspace_id"]
    pl = get_planning_environment()
    planning_env = PlanningEnvironmentRos.from_ros_msg(pl)
    slots = planning_env.get_unfinished_planned_items(place_id)
    
    slots = sorted(slots, key = lambda i: int((i.name).split("-")[1]))
    
    if len(slots)%2!=0:
        self.logger.info(f"当前规划不为偶数,检查是否已经被合并过")
        check_slot = slots[0]
        sku_dimension = list(check_slot.primitives[0].dimensions)
        sku_dimension = list(map(lambda x:round(x,3),sku_dimension))
        if sku_dimension == [0.6,0.4,0.23]:
            self.logger.info(f"已经被合并为大欧箱子,无需再合并")
            return "success"
        else:
            self.logger.info(f"无效的尺寸{sku_dimension}")
            raise f"无效的尺寸{sku_dimension}"
    else:    
        update_slots = copy.deepcopy(slots[0:2])
        
        new_slot = merge_items(update_slots)
        slots[0:2] = [new_slot]
        set_planned_items(place_id, slots, [])
        return "success"
