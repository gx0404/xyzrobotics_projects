from xyz_motion import SE3,pose_to_list,create_kinesolver,RobotDriver
from xyz_env_manager.client import get_planning_environment
from xyz_motion import PlanningEnvironmentRos
from py_xyz_robot.robot_config import get_robot_name

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
    grasp_plan = inputs.get("grasp_plan", None)
    object_poses = inputs.get("object_poses", None)
    tf_map_flange_list = inputs.get("tf_map_flange_list", None)
    
    
    if not gvm.get_variable("motion_payload", per_reference=True, default=None):
       planning_env_msg = get_planning_environment()
       planning_env = PlanningEnvironmentRos.from_ros_msg(planning_env_msg)
    else:
       last_payload = gvm.get_variable("motion_payload", per_reference=True, default=None)
       planning_env = PlanningEnvironmentRos.from_ros_msg(last_payload["planning_environment"])

    pick_workspace_id = grasp_plan.from_workspace_id
    place_workspace_id = grasp_plan.to_workspace_id
       
    if not grasp_plan:
        raise "需要连接grasp plan"
    if not((object_poses and not tf_map_flange_list) or (tf_map_flange_list and not object_poses)):
        raise "object_poses 和 tf_map_flange_list 必须连接并且只能连接一个"
    if object_poses:
        workspace_ros = planning_env.get_workspace_ros(place_workspace_id)          
                          
        tf_flange_tip = SE3(pose_to_list(grasp_plan.tip.tip_pose))
        tf_tip_object_list = []
        tf_base_object_list = []
        for tf_tip_object in grasp_plan.object_tip_transforms:
                tf_tip_object = SE3(pose_to_list(tf_tip_object))
                tf_tip_object_list.append(tf_tip_object)
        for pose_base_object in object_poses:
            tf_base_object = SE3(pose_base_object)
            tf_base_object_list.append(tf_base_object) 
        tf_base_flange_list = []
        for tf_base_object in tf_base_object_list:
            for tf_tip_object in tf_tip_object_list:
                tf_base_flange = (tf_base_object*tf_tip_object.inv())*tf_flange_tip.inv()
                tf_base_flange_list.append(tf_base_flange)
                
        work_space_dimensions = workspace_ros.get_dimensions()
        work_space_pose = workspace_ros.get_bottom_pose().xyz_quat
        
        tip_pose_z = tf_flange_tip.xyz_quat[2]
        #通过现有最高的箱子计算偏移
        container_items = planning_env.get_container_items(place_workspace_id)
        if container_items:
            check_items = filter_layer_items(container_items)
            space_pose_obj_z = check_items[0].origin.z+tip_pose_z+self.smart_data["sku_max_height"]+0.05
        else:
            space_pose_obj_z = work_space_pose[2]+tip_pose_z+self.smart_data["sku_max_height"]+0.05
            
        pose_base_flange = tf_base_flange_list[0].xyz_quat
                
        pose_xyz = pose_base_flange[0:2] + [space_pose_obj_z]        
        relative_pose = []        
        for index,item in enumerate(pose_xyz):
            relative_pose.append(item-pose_base_flange[index])
        outputs["relative_poses"] = [relative_pose+[0,0,0,1]]                   
        return "success"
    elif tf_map_flange_list:
        row = gvm.get_variable("row", per_reference=False, default=None)   
        if row==5:
            self.logger.info("row is 5")
            outputs["near_relative_poses"] = [[0, 0, 0.1, 0, 0, 0, 1]]
        elif row==9:
            self.logger.info("row is 9")    
            outputs["near_relative_poses"] = [[0, 0, 0.07, 0, 0, 0, 1]]
        else:
            raise "无效的row"       
        workspace_ros = planning_env.get_workspace_ros(pick_workspace_id)
        tf_flange_tip = SE3(pose_to_list(grasp_plan.tip.tip_pose))

        work_space_pose = workspace_ros.get_bottom_pose().xyz_quat
        tip_pose_z = tf_flange_tip.xyz_quat[2]
        
        #通过现有最高的箱子计算偏移
        container_items = planning_env.get_container_items(pick_workspace_id)
        if container_items:
            check_items = filter_layer_items(container_items)
            space_pose_obj_z = check_items[0].origin.z+tip_pose_z+self.smart_data["sku_max_height"]+0.05
        else:
            space_pose_obj_z = work_space_pose[2]+tip_pose_z+self.smart_data["sku_max_height"]+0.05
        
        relative_pose_list = []
        pose_base_flange = tf_map_flange_list[0]
        pose_xyz = pose_base_flange[0:2] + [space_pose_obj_z]
        relative_pose = []        
        for index,item in enumerate(pose_xyz):
            relative_pose.append(item-pose_base_flange[index])         
        relative_pose+=[0,0,0,1] 
        
        relative_pose_list.append(relative_pose)
        outputs["relative_poses"] = relative_pose_list
        return "success"   
       