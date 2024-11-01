from xyz_motion import SE3,pose_to_list,create_kinesolver
from xyz_env_manager.client import get_planning_environment
from xyz_motion import PlanningEnvironmentRos
def execute(self, inputs, outputs, gvm):
    self.logger.info("Hello {}".format(self.name))
    grasp_plan = inputs.get("grasp_plan", None)
    object_poses = inputs.get("object_poses", None)
    tf_map_flange_list = inputs.get("tf_map_flange_list", None)
    
    
    #获取环境
    planning_env_msg = get_planning_environment()
    planning_env = PlanningEnvironmentRos.from_ros_msg(planning_env_msg)
    place_workspace_id = grasp_plan.to_workspace_id
    workspace_ros = planning_env.get_workspace_ros(place_workspace_id)
    
    if not place_workspace_id in ["4"]:
        outputs["relative_poses"] = [[0,0,0.6,0,0,0,1],[0,0,0.3,0,0,0,1]]                   
        return "success"        
    
    if not grasp_plan:
        raise "需要连接grasp plan"
    if not((object_poses and not tf_map_flange_list) or (tf_map_flange_list and not object_poses)):
        raise "object_poses 和 tf_map_flange_list 必须连接并且只能连接一个"
    if object_poses:
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
        space_pose_obj_z = work_space_pose[2]+work_space_dimensions[2]+tip_pose_z+self.smart_data["sku_max_height"]
        pose_base_flange = tf_base_flange_list[0].xyz_quat
                
        pose_xyz = pose_base_flange[0:2] + [space_pose_obj_z]        
        relative_pose = []        
        for index,item in enumerate(pose_xyz):
            relative_pose.append(item-pose_base_flange[index])
        outputs["relative_poses"] = [relative_pose+[0,0,0,1]]                   
        return "success"
