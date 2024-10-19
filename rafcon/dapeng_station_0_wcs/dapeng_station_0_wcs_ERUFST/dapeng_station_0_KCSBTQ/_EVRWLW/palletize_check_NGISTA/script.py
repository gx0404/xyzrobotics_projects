from xyz_motion import PlanningEnvironmentRos,pose_to_list,SE3
from xyz_env_manager.client import get_planning_environment
import numpy as np
from xyz_env_manager.client import set_planned_items,clear_planned_items
from xyz_env_manager.client import clear_container_all_items,add_container_items
import tf.transformations as tfm
import copy
from xyz_env_manager.msg import Pose





def execute(self, inputs, outputs, gvm):
    self.logger.info("Hello {}".format(self.name))
    planning_env_msg = get_planning_environment()
    planning_env = PlanningEnvironmentRos.from_ros_msg(planning_env_msg)

    #获取托盘料箱ID信息
    new_pose_dict = inputs["pose_dict"]

        
    #获取合托缓存区上所有箱子
    container_items = planning_env.get_container_items("2")

    #获取拣配托盘坐标
    workspace_ros = planning_env.get_workspace_ros("2")
    tf_base_space = workspace_ros.get_bottom_pose()


    #匹配当前箱子条码     
    for container_item in container_items:
        tf_space_box_real = (tf_base_space.inv())*SE3(pose_to_list(container_item.origin))
        space_box_pose_real = tf_space_box_real.xyz_quat
        self.logger.info(f"拣配视觉料箱坐标XYZ坐标为 {space_box_pose_real[0:3]}")
        for key,values in new_pose_dict.items():    
            check_list = list(filter(lambda x:abs(space_box_pose_real[x]-values[x])<0.07,[0,1]))  
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
            #container_item.additional_info.values[index] = pallet_tote_data[box_id]["barcode"]     
        else:    
            container_item.additional_info.keys.append("barcode")
            #container_item.additional_info.values.append(pallet_tote_data[box_id]["barcode"]) 
        #更新to_ws到环境,主要是因为一致性,实际用处不大       
        if "to_ws" in container_item.additional_info.keys:
            index = container_item.additional_info.keys.index("to_ws")
            container_item.additional_info.values[index] = ""    
        else:    
            container_item.additional_info.keys.append("to_ws")
            container_item.additional_info.values.append("")                 
             

    #获取合托缓存区放置规划箱子
    plan_items = planning_env.get_unfinished_planned_items("2")  

    check_plan_list = []
    for container_item in container_items:
        tf_box_real = SE3(pose_to_list(container_item.origin))
        #通过视觉的箱子找到放置规划的箱子
        remove_plan_item = list(filter(lambda x:np.allclose(pose_to_list(x.origin)[0:3],tf_box_real.xyz_quat[0:3],atol=0.08),plan_items))
        if not remove_plan_item:
            self.logger.info(f"合托缓存区上没有找到匹配的箱子{container_item.name}")
            raise f"合托缓存区上没有找到匹配的箱子"
        remove_plan_item = remove_plan_item[0]    
        #移除放置规划的箱子
        plan_items.remove(remove_plan_item)
        check_plan_list.append(remove_plan_item)

    new_pick_plan_items = []
    for plan_item in plan_items:
        tf_space_box_real = (tf_base_space.inv())*SE3(pose_to_list(plan_item.origin))
        space_box_pose_real = tf_space_box_real.xyz_quat
        self.logger.info(f"放置规划XYZ坐标为 {space_box_pose_real[0:3]}")
        for key,values in new_pose_dict.items():    
            check_list = list(filter(lambda x:abs(space_box_pose_real[x]-values[x])<0.07,[0,1]))  
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
    if len(check_plan_list)!=len(container_items):
        self.logger.info(f"拣配托盘放置规划匹配到{len(check_plan_list)}个箱子,实际视觉有{len(container_items)}个箱子")   
        raise f"拣配托盘放置规划匹配箱子和实际视觉箱子数量不匹配"
    
    clear_container_all_items("2")
    add_container_items("2",container_items)
    set_planned_items("2",new_pick_plan_items,[])
    return "success"
