from xyz_env_manager.client import get_planning_environment, add_full_padding_exclude, remove_all_full_padding, add_full_padding_workspace
from xyz_env_manager.client import modify_primitive_group_of_environment
from xyz_motion import PlanningEnvironmentRos
from rafcon.utils.helper_function import find_the_nearest_state_in_concurrency_state
def execute(self, inputs, outputs, gvm):
    """ 
    This state changes free workspaces to collision objects. 

    Args:
        Inputs Data:
            grasp_plan (xyz_env_manager.msg.GraspPlan): None
                A environment manager message represents which obejct to be picked, which tool to pick, and the relative relationship between tip and object origin.
            trajectory (dict): None
                same payload as the input with only "executed_name" has waypoints traversed in this state being appended.


        Outputs Data:

        Gvm: 
            motion_payload (dict) (set/get):
                A compound payload to store all necessary information for motion planning, environment updating, and driver execution

    Properties Data:
        comment, (unicode): "update_workspace_collision"
            Recommend to assign the name of this motion that easy to recognize
        other_non_collision_workspace_ids, (list): []
            Besides pick and place workspaces, these states are also considered as non-collision object
    """
         
    self.logger.info("Running {}({})({})".format(self.name, self.unique_id, self.smart_data["comment"]))
    grasp_plan = inputs.get("grasp_plan", None)
    trajectory = inputs.get("trajectory", None)
    concurrency_state = find_the_nearest_state_in_concurrency_state(self)
    other_non_collision_workspace_ids = self.smart_data["other_non_collision_workspace_ids"]
    other_non_collision_workspace_ids+=["6"]
    other_collision_workspace_ids = self.smart_data["other_collision_workspace_ids"]

    if len(list(set(other_non_collision_workspace_ids).intersection(set(other_collision_workspace_ids)))) != 0:
        self.logger.warnning("非障碍物的工作空间ID与指定障碍物的工作空间的ID有重复")
    

    def update_planning_collision(grasp_plan):
        if not gvm.get_variable("motion_payload", per_reference=True, default=None):
            last_payload = {}
            planning_env_msg = get_planning_environment()
            planning_env = PlanningEnvironmentRos.from_ros_msg(planning_env_msg)
        else:
            last_payload = gvm.get_variable("motion_payload", per_reference=True, default=None)
            planning_env = PlanningEnvironmentRos.from_ros_msg(last_payload["planning_environment"])
        
        planning_pick_workspace_id = grasp_plan.from_workspace_id
        planning_place_workspace_id = grasp_plan.to_workspace_id
        non_collision_workspace_ids = [planning_pick_workspace_id] + [planning_place_workspace_id] + other_non_collision_workspace_ids
        planning_env.remove_all_full_padding()
        planning_env.add_full_padding_exclude(non_collision_workspace_ids)                         
        for ws_id in other_collision_workspace_ids:
            planning_env.add_full_padding_workspace(ws_id)
            
        planning_env_msg = PlanningEnvironmentRos.to_ros_msg(planning_env)
        
        check_padding_list = [f"workspace_{i}_full_padding_pg" for i in range(3,4)]
        for collision_object in planning_env_msg.collision_objects:
            if collision_object.name in check_padding_list:
                dimensions = list(collision_object.primitives[0].dimensions)
                enlarge_length = 1.3
                dimensions[1]+=enlarge_length
                dimensions[0]-=0.05
                collision_object.primitives[0].dimensions=tuple(dimensions)   
                # if collision_object.origin.x>-0.5 and collision_object.origin.y>0:
                #     collision_object.origin.y +=1/2
                # elif collision_object.origin.x>-0.5 and collision_object.origin.y<0:
                #     collision_object.origin.y -=1/2
                # elif collision_object.origin.x<-0.5:
                #     collision_object.origin.x -=1/2                        
        last_payload["planning_environment"] = planning_env_msg
        gvm.set_variable("motion_payload", last_payload, per_reference=True)
    
    def update_visual_collision(trajectory):
        visual_pick_workspace_id = trajectory["grasp_plan"].from_workspace_id
        visual_place_workspace_id = trajectory["grasp_plan"].to_workspace_id
        non_collision_workspace_ids = [visual_pick_workspace_id] + [visual_place_workspace_id] + other_non_collision_workspace_ids
        remove_all_full_padding()
        add_full_padding_exclude(non_collision_workspace_ids)
        for ws_id in other_collision_workspace_ids:
            add_full_padding_workspace(ws_id)
        planning_env_msg = get_planning_environment()
        check_padding_list = [f"workspace_{i}_full_padding_pg" for i in range(3,4)]
        for collision_object in planning_env_msg.collision_objects:
            if collision_object.name in check_padding_list:
                dimensions = list(collision_object.primitives[0].dimensions)
                enlarge_length = 1.3
                dimensions[1]+=enlarge_length
                dimensions[0]-=0.05
                collision_object.primitives[0].dimensions=tuple(dimensions)
                # if collision_object.origin.x>-0.5 and collision_object.origin.y>0:
                #     collision_object.origin.y +=1/2
                # elif collision_object.origin.x>-0.5 and collision_object.origin.y<0:
                #     collision_object.origin.y -=1/2
                # elif collision_object.origin.x<-0.5:
                #     collision_object.origin.x -=1/2      
                modify_primitive_group_of_environment(collision_object)               
    # update collisions in planning environment 
    if grasp_plan and not trajectory:
        update_planning_collision(grasp_plan)
        self.logger.info("更新规划环境中的工作空间障碍物")

    # update collisions in visual(real) environment
    elif trajectory and not grasp_plan:
        update_visual_collision(trajectory)
        self.logger.info("更新可视化环境中的工作空间障碍物")
    
    # update collisions in all environments
    elif trajectory and grasp_plan:
        if concurrency_state:
            raise Exception("在并行中更新工作空间障碍物需要区分规划环境和可视化环境！")
        update_planning_collision(grasp_plan)
        update_visual_collision(trajectory)
        self.logger.info("更新规划环境和可视化环境中的工作空间障碍物")

    
    else:
        raise Exception("没有合法的输入来更新工作空间障碍物！")
    
    return "success"
