from tabnanny import check
from xyz_env_manager.client import get_planning_environment
from xyz_motion import PlanningEnvironmentRos,pose_to_list,SE3
import copy
import math
def execute(self, inputs, outputs, gvm):
    self.logger.info("Hello {}".format(self.name))

    planning_env_msg = get_planning_environment()
    planning_env = PlanningEnvironmentRos.from_ros_msg(planning_env_msg)  
        
    space_env = planning_env.get_workspace_ros(self.smart_data["place_id"])
    tf_base_space = space_env.get_bottom_pose()
    if self.smart_data["place_id"]=="6":
        outputs["place_box_id"] = {
            "id":1
        }   
        return "success"
    pose_dict = inputs["pose_dict"]    
    grasp_plan = inputs["grasp_plan"]
    #通过放置物体id，在unfinished items找到放置位置
    place_pose_items = []
    planned_items_ids = grasp_plan.planned_items_ids
    unfinished_items = planning_env.get_unfinished_planned_items(self.smart_data["place_id"])
    for unfinished_item in unfinished_items:
        for plan_item_id in planned_items_ids:
            if unfinished_item.name == plan_item_id:
                place_pose_items.append(unfinished_item)  
                self.logger.info(f"匹配到纸箱id为{plan_item_id}")
                self.logger.info(f"纸箱放置位姿为\n{unfinished_item.origin}")
    #放置位置匹配位置号            
    box_id_list = {}

    for place_pose in place_pose_items:
        box_id_dict = {}       
        tf_space_box_real = (tf_base_space.inv())*SE3(pose_to_list(place_pose.origin))        
        space_box_pose_real = tf_space_box_real.xyz_quat

        self.logger.info(f"实际放置XYZ坐标为 {space_box_pose_real[0:3]}")
        for key,values in pose_dict.items():    
            check_list = list(filter(lambda x:abs(space_box_pose_real[x]-values[x])<0.07,[0,1]))  
            check_list_z = list(filter(lambda x:abs(space_box_pose_real[x]-values[x])<0.05,[2]))           
            if len(check_list)==2 and check_list_z:
                box_id = key
                break
            else:
                box_id = -1
                                   
        if box_id<0:
            self.logger.error("未能匹配上位置号")
            raise "未能匹配上位置号" 
        box_id_dict["name"] = place_pose.name     
        box_id_dict["pose"] = space_box_pose_real
        box_id_dict["id"] = box_id
        box_id_list[str(box_id)] = box_id_dict       
    outputs["place_box_id"] = box_id_list

    self.logger.info("place position id is \n{}".format(box_id_list))     
    
    return "success"
