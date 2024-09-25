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
    other_non_collision_workspace_ids = self.smart_data["other_non_collision_workspace_ids"]
    other_collision_workspace_ids = self.smart_data["other_collision_workspace_ids"]
    
    if len(list(set(other_non_collision_workspace_ids).intersection(set(other_collision_workspace_ids)))) != 0:
        self.logger.warnning("非障碍物的工作空间ID与指定障碍物的工作空间的ID有重复")
    

    non_collision_workspace_ids = other_non_collision_workspace_ids
    remove_all_full_padding()
    add_full_padding_exclude(non_collision_workspace_ids)
    for ws_id in other_collision_workspace_ids:
        add_full_padding_workspace(ws_id)
        
    planning_env_msg = get_planning_environment()
    check_padding_list = [f"workspace_{i}_full_padding_pg" for i in range(0,1)]
    for collision_object in planning_env_msg.collision_objects:
        if collision_object.name in check_padding_list:
            dimensions = list(collision_object.primitives[0].dimensions)
            enlarge_ = 0.22
            dimensions[0]+=enlarge_
            dimensions[1]+=enlarge_
            dimensions[2]*=4
            collision_object.primitives[0].dimensions=tuple(dimensions)
            # if collision_object.origin.x>-0.5 and collision_object.origin.y>0:
            #     collision_object.origin.y +=1/2
            # elif collision_object.origin.x>-0.5 and collision_object.origin.y<0:
            #     collision_object.origin.y -=1/2
            # elif collision_object.origin.x<-0.5:
            #     collision_object.origin.x -=1/2      
            modify_primitive_group_of_environment(collision_object)        
    return "success"
