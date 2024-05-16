from xyz_motion import PlanningEnvironmentRos,pose_to_list,SE3
from xyz_env_manager.client import get_planning_environment
import tf.transformations as tfm
import numpy as np
from xyz_env_manager.msg import Pose
from xyz_env_manager.client import clear_container_all_items,add_container_items

def execute(self, inputs, outputs, gvm):
    self.logger.info("Hello {}".format(self.name))
    planning_env_msg = get_planning_environment()
    planning_env = PlanningEnvironmentRos.from_ros_msg(planning_env_msg)
    #获取笼车托盘所有箱子
    place_container_items = planning_env.get_container_items("2")
    #获取笼车托盘信息
    pallet_tote_data_2 = inputs["pallet_tote_data_2"]
    if len(place_container_items)!=len(pallet_tote_data_2):
        self.logger.info(f"笼车托盘数据长度和视觉箱子数量不一致")
        raise f"笼车托盘数据长度和视觉箱子数量不一致"
    
    #获取笼车托盘托盘坐标
    place_workspace_ros = planning_env.get_workspace_ros("2")
    tf_base_place_space = place_workspace_ros.get_bottom_pose()
    pose_dict = inputs["pose_dict"]
    #匹配当前箱子条码     
    for container_item in place_container_items:
        tf_base_box_real = SE3(pose_to_list(container_item.origin))
        tf_space_box_real = (tf_base_place_space.inv())*tf_base_box_real
        space_box_pose_real = tf_space_box_real.xyz_quat
        self.logger.info(f"缓存视觉料箱坐标XYZ坐标为 {space_box_pose_real[0:3]}")
        for key,values in pose_dict.items():    
            check_list = list(filter(lambda x:abs(space_box_pose_real[x]-values[x])<0.07,[0,1]))  
            check_list_z = list(filter(lambda x:abs(space_box_pose_real[x]-values[x])<0.05,[2]))           
            if len(check_list)==2 and check_list_z:
                box_id = key
                break
            else:
                box_id = -1       
        if box_id<0:
            self.logger.error("未能匹配上位置号,请检查视觉数据和缓存数据")
            raise "未能匹配上位置号,请检查视觉数据和缓存数据"
        else:    
            self.logger.info(f"匹配到位置id为{box_id}")    
        #添加box_id
        box_id = str(box_id) 
        #添加信息到环境
        if box_id in pallet_tote_data_2.keys():
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
                container_item.additional_info.values[index] = pallet_tote_data_2[box_id]["barcode"]       
            else:    
                container_item.additional_info.keys.append("barcode")
                container_item.additional_info.values.append(pallet_tote_data_2[box_id]["barcode"]) 
            #更新to_ws到环境,主要是因为一致性,实际用处不大       
            if "barcode" in container_item.additional_info.keys:
                index = container_item.additional_info.keys.index("to_ws")
                container_item.additional_info.values[index] = ""    
            else:    
                container_item.additional_info.keys.append("to_ws")
                container_item.additional_info.values.append("")   
                    
        else:
            self.logger.info(f"托盘数据缺少位置号{box_id}")
            raise f"托盘数据缺少位置号"    
    clear_container_all_items("2")
    add_container_items("2",place_container_items)

    return "success"
