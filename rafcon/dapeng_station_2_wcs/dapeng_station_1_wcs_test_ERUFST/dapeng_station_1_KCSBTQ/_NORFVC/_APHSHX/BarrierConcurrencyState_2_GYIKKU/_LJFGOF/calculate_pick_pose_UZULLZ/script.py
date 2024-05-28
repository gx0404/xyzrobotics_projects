from xyz_motion import PlanningEnvironmentRos,SE3,pose_to_list
from xyz_motion import AllRobotToolStatesRos
from xyz_env_manager.client import get_planning_environment
from xyz_env_manager.client import get_all_robot_tool_states
def execute(self, inputs, outputs, gvm):
    self.logger.info("Hello {}".format(self.name))
    if not gvm.get_variable("motion_payload", per_reference=True, default=None):
        planning_env_msg = get_planning_environment()
        planning_env = PlanningEnvironmentRos.from_ros_msg(planning_env_msg)
    else:
        last_payload = gvm.get_variable("motion_payload", per_reference=True, default=None)
        planning_env = PlanningEnvironmentRos.from_ros_msg(last_payload["planning_environment"])
    workspace_id = self.smart_data["workspace_id"]
    
    #通过抓取计算得到的路径，指定抓取箱子
    plan_path = gvm.get_variable("plan_path", per_reference=False, default=None)
    pick_item_id = plan_path[0]    
    items = planning_env.get_container_items(workspace_id)
    for item in items:
        if item.additional_info.values[-3]==pick_item_id:
            break
    #获取当前抓取箱子坐标
    tf_map_object = SE3(pose_to_list(item.origin))
    outputs["tf_map_object"] = tf_map_object.xyz_quat
    tf_map_object = tf_map_object*SE3([0,0,-0.035,0,0,0,1])   
           
    all_robot_states = AllRobotToolStatesRos.from_ros_msg(get_all_robot_tool_states())
    planning_robot_ros = all_robot_states.get_robot_ros("0")
    planning_robot_ros.detach_object()
    
    #获取抓取法兰所有姿态
    tf_tip_object_list = [[0,0,0,1,0,0,0],[0,0,0,0,1,0,0]]   
    tf_map_flange_list = []  
    
    for tf_tip_object in tf_tip_object_list:
        tf_tip_object = SE3(tf_tip_object)
        tf_map_flange = tf_map_object * tf_tip_object.inv() * planning_robot_ros.get_active_tool().get_tf_endflange_tip("tip_0").inv()
        tf_map_flange_list.append(tf_map_flange.xyz_quat) 
        
    #添加抓取姿态到全局变量
    gvm.set_variable("tf_map_flange_list", tf_map_flange_list, per_reference=False)  

    # place_ws_id = item.additional_info.values[-1]   
    place_ws_id = "1"
    slots = planning_env.get_unfinished_planned_items(place_ws_id)
    object_pose = pose_to_list(slots[0].origin)  
    object_pose_list = []
    object_pose_list.append(object_pose) 
    object_pose_list.append((SE3(object_pose)*SE3([0,0,0,0,0,1,0])).xyz_quat) 
    outputs["object_poses"] = object_pose_list         
    return "success"
