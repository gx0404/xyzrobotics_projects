import numpy as np
import copy
from xyz_motion.common import Rn
from xyz_motion import SE3, pose_to_list
from py_xyz_mf import MovementFlow, MoveType, SolverConfig, LogLevel, TrajectorySolverConfig, InterpConfig
from xyz_motion import RobotDriver
from xyz_env_manager.msg import AttachedCollisionObject
from xyz_motion import PlanningEnvironmentRos
from xyz_motion import pose_to_list
from xyz_env_manager.client import get_planning_environment
from xyz_env_manager.client import get_all_robot_tool_states
from xyz_motion import AllRobotToolStatesRos
from xyz_motion import SafetyDistance
from xyz_motion.solvers import WoptSolverConfig, AStarConfig, SmartPlannerOptions

def execute(self, inputs, outputs, gvm):
    """ 
    This state initialize the process of motion planning and receive a grasp plan to produce pick poses if this series of motion is for a pick&place task.
    """
    self.logger.info("Running {}({})".format(self.name, self.state_id))
    self.logger.info(self.smart_data["comment"])
    
    # create a empty motion payload
    motion_payload = {}
    # TODO gvm of planning_environment and robot
    if not gvm.get_variable("motion_payload", per_reference=True, default=None):
        ## TODO delele this if we use gvm
        planning_environment_ros = PlanningEnvironmentRos.from_ros_msg(get_planning_environment()) 
        motion_payload["planning_environment"] = planning_environment_ros.to_ros_msg()
    else:
        last_payload = gvm.get_variable("motion_payload", per_reference=True, default=None)
        motion_payload["planning_environment"] = last_payload["planning_environment"]

    # preparing collision workspace
    planning_environment_ros = copy.deepcopy(PlanningEnvironmentRos.from_ros_msg(motion_payload["planning_environment"]))

    all_robot_states = AllRobotToolStatesRos.from_ros_msg(get_all_robot_tool_states())
    planning_robot_ros = all_robot_states.get_robot_ros(self.smart_data["0-config-basic"]["robot_id"])
    planning_robot_ros.detach_object()
    if self.smart_data["3-config-robot"]["moving_base"]["enable"]:
        def normalized_xyz_quat(xyz_quat, tolerance=0.001):
            import math
            dx, dy, dz, qx, qy, qz, qw = xyz_quat
            square_of_quat_length = qx * qx + qy * qy + qz * qz + qw * qw
            if abs(square_of_quat_length - 1) > tolerance:
                raise Exception(
                    f"Invalid quaternion [{qx}, {qy}, {qz}, {qw}], "
                    + f"the square of length {square_of_quat_length} is not close enough to 1 "
                    + f"(tolerance: {tolerance})"
                )
            quat_length = math.sqrt(square_of_quat_length)
            return [dx, dy, dz, qx / quat_length, qy / quat_length, qz / quat_length, qw / quat_length]
        planning_robot_ros.set_base_transform(SE3(normalized_xyz_quat(self.smart_data["3-config-robot"]["moving_base"]["tf_base_robot"])))
    
    if self.smart_data["3-config-robot"]["active_tip_name"]:
        planning_robot_ros.set_active_tip_name(self.smart_data["3-config-robot"]["active_tip_name"])

    tf_map_flange_list = []
        
    if inputs["grasp_plan"]:
        grasp_plan = inputs["grasp_plan"]
        motion_payload["grasp_plan"] = grasp_plan
        planning_environment_ros.set_from_workspace_id(grasp_plan.from_workspace_id)
        planning_environment_ros.set_to_workspace_id(grasp_plan.to_workspace_id)
        
        attached_collision_object = AttachedCollisionObject()
        attached_collision_object.reference_tip_name = grasp_plan.tip.name
        attached_collision_object.reference_object_name = grasp_plan.reference_object_name
        attached_collision_object.objects = grasp_plan.objects
        attached_collision_object.from_workspace_id = grasp_plan.from_workspace_id
        attached_collision_object.to_workspace_id = grasp_plan.to_workspace_id
        motion_payload["attached_collision_object"] = attached_collision_object

        for pg in attached_collision_object.objects:
            if pg.name == attached_collision_object.reference_object_name:
                tf_map_object = SE3(pose_to_list(pg.origin))
                break
        else:
            raise Exception("Cannot find reference_object_name: {} in grasp plan!".format(attached_collision_object.reference_object_name))        


        for tf_tip_object in grasp_plan.object_tip_transforms:
            tf_tip_object = SE3(pose_to_list(tf_tip_object))
            tf_map_flange = tf_map_object * tf_tip_object.inv() * planning_robot_ros.get_active_tool().get_tf_endflange_tip(grasp_plan.tip.name).inv()
            tf_map_flange_list.append(tf_map_flange.xyz_quat)

        # TODO: Duplicates removal should be done outside motion planner
        # remove tf_map_flange_list duplicates
        tf_map_flange_list = np.array(tf_map_flange_list)
        tf_index = np.unique(tf_map_flange_list.round(decimals=4), axis=0, return_index = True)[1]
        tf_index.sort()
        tf_map_flange_list = [tf_map_flange_list[index].tolist() for index in tf_index]
        if grasp_plan.transform_groups:
            tf_map_flange_list.append(tuple(grasp_plan.transform_groups[index] for index in tf_index))
    else:
        self.logger.info("Grasp plan is empty!")
        
    # TODO: may need to read status from the last driver execution in the future
    # TODO: deal with no item on robot at any start point of motion planner.
    #添加抓取姿态到全局变量
    gvm.set_variable("tf_map_flange_list", tf_map_flange_list, per_reference=False) 
    gvm.set_variable("attached_collision_object", attached_collision_object, per_reference=False) 
    
    return "success"
