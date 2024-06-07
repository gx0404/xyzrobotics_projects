import time
from visualization_msgs.msg import Marker, MarkerArray
import rospy 
from xyz_env_manager.client import get_planning_environment, clear_container_all_items, clear_planned_items, remove_all_full_padding
from xyz_env_manager.client import clear_attached_collision_object, remove_bottom_padding_workspace, remove_full_padding_workspace
from xyz_motion import PlanningEnvironmentRos

def execute(self, inputs, outputs, gvm):
    """ 
    This state clear all items in specified workspaces. If not workspace id is specified, it will clear items in all workspaces.
    Items are things that are assigned in a workspace and should be dynamic items which sent by WCS orders or vision.

    Please Notice that we delete motion_playload in gvm, so this means we clear planning env.

    Args:
        Inputs Data:
            None

        Outputs Data:
            None

        Gvm: 
            motion_payload (dict) set:
                A compound payload to describe the planning environment use so far. It set to None(Clear).
            

    Properties Data:
        workspaces, (list): [].
            A list of unicodes represent the id of workspaces need to be cleared.

        clear_all_workspaces, (bool): False.
            A flag indicated we need clear all workspace's items and planned items. 
        clear_all_full_padding, (bool): True.
            A flag that clear full padding of workspace
        clear_pickable, (dict): { 
                                "enable": true, 
                                "robot_id": "0"
                                }.
            A flag indicated we need clear robot's pickable items.

    Outcomes:
        0: success
        -1: aborted
        -2: preempted
    """
    self.logger.info("Running {}({})({})".format(self.name, self.unique_id, self.smart_data["comment"]))
    target_workspaces = self.smart_data["workspaces"] if self.smart_data["is_workspace_list"] else [self.smart_data["workspace"]]

    if self.smart_data["clear_motion_payload"] or not gvm.get_variable("motion_payload", per_reference=True, default=None):
        gvm.set_variable("motion_payload", None, per_reference=True)
        if self.smart_data["clear_all_padding"]:
            remove_all_full_padding()
        workspace_env_msg = get_planning_environment()
        for workspace_msg in workspace_env_msg.workspaces:
            workspace_id = workspace_msg.workspace_id
            if self.smart_data['clear_all_padding']:
                remove_bottom_padding_workspace(workspace_id)
            if self.smart_data["clear_all_workspaces"] or workspace_id in target_workspaces:
                if workspace_id in self.smart_data["un_workspaces"]:
                    pass
                else:
                    clear_container_all_items(workspace_id)
                    clear_planned_items(workspace_id)
                if not self.smart_data['clear_all_padding']:
                    remove_full_padding_workspace(workspace_id)
                    remove_bottom_padding_workspace(workspace_id)

    else:
        last_payload = gvm.get_variable("motion_payload", per_reference=True, default=None)
        workspace_env_msg = last_payload["planning_environment"]
        planning_env = PlanningEnvironmentRos.from_ros_msg(workspace_env_msg)
        if self.smart_data["clear_all_padding"]:
            remove_all_full_padding()
            planning_env.remove_all_full_padding()
        for workspace_msg in workspace_env_msg.workspaces:
            workspace_id = workspace_msg.workspace_id
            if self.smart_data['clear_all_padding']:
                remove_bottom_padding_workspace(workspace_id)
                planning_env.remove_bottom_padding_workspace(workspace_id)
            if self.smart_data["clear_all_workspaces"] or workspace_id in target_workspaces:
                if workspace_id in self.smart_data["un_workspaces"]:
                    pass
                else:
                    clear_container_all_items(workspace_id)
                    clear_planned_items(workspace_id)
                planning_env.clear_container_all_items(workspace_id)
                planning_env.clear_planned_items(workspace_id)
                if not self.smart_data['clear_all_padding']:
                    remove_full_padding_workspace(workspace_id)
                    remove_bottom_padding_workspace(workspace_id)
                    planning_env.remove_full_padding_workspace(workspace_id)
                    planning_env.remove_bottom_padding_workspace(workspace_id)
        last_payload["planning_environment"] = PlanningEnvironmentRos.to_ros_msg(planning_env)
        gvm.set_variable("motion_payload", last_payload, per_reference=True)

    if self.smart_data["clear_pickable"]["enable"]:
        clear_attached_collision_object(self.smart_data["clear_pickable"]["robot_id"], clear_all = True)


    return "success"