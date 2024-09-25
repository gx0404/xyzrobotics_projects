import numpy as np
import copy
from xyz_motion.common import Rn
from xyz_motion import SE3, pose_to_list
from py_xyz_mf import MovementFlow, MoveType, SolverConfig, LogLevel, TrajectorySolverConfig, InterpConfig
from xyz_motion import RobotDriver
from xyz_env_manager.msg import AttachedCollisionObject
from xyz_motion import PlanningEnvironmentRos,RobotRos
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

    # all_robot_states = AllRobotToolStatesRos.from_ros_msg(get_all_robot_tool_states())
    # planning_robot_ros = all_robot_states.get_robot_ros(self.smart_data["0-config-basic"]["robot_id"])
    # planning_robot_ros.detach_object()
    our_robot_msg = gvm.get_variable("our_robot_msg", per_reference=True, default=None)
    planning_robot_ros = RobotRos.from_ros_msg(our_robot_msg)
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
        
    outputs["tf_map_flange_list"] = tf_map_flange_list
    ## TODO input robot state rather than q0
    if inputs["q0"]:
        q0 = inputs["q0"]
    else:
        rob = RobotDriver(int(self.smart_data["0-config-basic"]["robot_id"]))
        ## TODO Let robot gives us a flag of robot is moving.
        q0 = list(rob.get_joints())

    config = SolverConfig()
    config.timeout = self.smart_data["0-config-basic"]["timeout"]
    config.log_level = getattr(LogLevel, self.smart_data["0-config-basic"]["log_level"])
    config.enable_early_check = self.smart_data["0-config-basic"]["enable_early_check"]
    config.keep_conf = False # Solvers like Wopt is free to change robot conf to find a solution
    config.guess_cache_size = 200
    config.choice_cache_size = 10
    config.dump_failure = self.smart_data["0-config-basic"]["dump_failure"]
    config.dump_success = self.smart_data["0-config-basic"]["dump_success"]
    config.derived_node_env = True
    
    interp_config = InterpConfig()
    for key, value in self.smart_data["1-config-interp"].items():
        interp_config.__setattr__(key, value)
    motion_payload["interp_config"] = interp_config

    ## wopt config setting
    wopt_config = WoptSolverConfig()
    wopt_config.dcheck = [self.smart_data["2-config-solver"]["dcheck"]]
    wopt_config.dsafe = [self.smart_data["2-config-solver"]["dsafe"]["midway"]]
    wopt_config.limit_joint = True
    wopt_config.merit_error_coeff = 5.0
    wopt_config.number_of_swept_interpolations = 4
    wopt_config.trust_box_size = 0.5
    wopt_config.do_log = True
    wopt_config.log_directory = "/home/xyz/xyz_log/wopt_log"
    wopt_config.post_solve_simplification = self.smart_data["2-config-solver"]["simplify"]
        
    astar_config = AStarConfig()
    smart_planner_options = SmartPlannerOptions()
    smart_planner_options.do_log = True
    smart_planner_options.log_directory = "/home/xyz/xyz_log/smart_planner_log"
    smart_planner_options.post_solve_simplification = self.smart_data["2-config-solver"]["simplify"]
    
    solver_config = TrajectorySolverConfig()
    solver_config.wopt_config = wopt_config
    solver_config.astar_config = astar_config
    solver_config.smart_planner_config = smart_planner_options
    solver_config.safety_distance.at_start = self.smart_data["2-config-solver"]["dsafe"]["at_start"]
    solver_config.safety_distance.at_goal = self.smart_data["2-config-solver"]["dsafe"]["at_goal"]
    solver_config.safety_distance.midway = self.smart_data["2-config-solver"]["dsafe"]["midway"]
    solver_config.check_margin_offset = self.smart_data["2-config-solver"]["dcheck"] - self.smart_data["2-config-solver"]["dsafe"]["midway"]

    solver_config.step_num = self.smart_data["2-config-solver"]["step_num"]
    motion_payload["solver_config"] = solver_config
    
    movement_flow = MovementFlow(planning_robot_ros, planning_environment_ros, config)
    ## Add initial node to movement flow
    res = movement_flow.add_independent_node("initial", 
                                        "", [],
                                        [np.array(q0)], 
                                        [], 
                                        False, False, "tail",
                                        np.array([0, 0, 0]),
                                        [],
                                        "",
                                        MoveType.MoveJ, 
                                        interp_config,
                                        solver_config
                                        )
    if not res:
        raise Exception("init_movement_flow failed! Please check movement flow log!")
                                        
    if self.smart_data["3-config-robot"]["initial_pickable"]:
        if not inputs["execution_payload"]:
            raise Exception("Need an execution_payload to generate initial pickable!")
        if inputs["grasp_plan"]:
            raise Exception("Cannot calculate grasp plan with initial pickable!")
        attached_collision_object = inputs["execution_payload"]["attached_collision_object"]
        ksolver = planning_robot_ros.get_kinematics_solver()
        tf_map_flange = ksolver.compute_fk(q0)
        tf_flange_tip = planning_robot_ros.get_active_tool().get_tf_endflange_tip(attached_collision_object.reference_tip_name)
        tf_tip_object = SE3(pose_to_list(attached_collision_object.origin_tip_transform))
        tf_map_new_ref_object = tf_map_flange * tf_flange_tip * tf_tip_object
        
        tf_map_old_reference_object = None
        for env_object in attached_collision_object.objects:
            if env_object.name == attached_collision_object.reference_object_name:    
                tf_map_old_reference_object = SE3(pose_to_list(env_object.origin))

        if not tf_map_old_reference_object:
            raise Exception("Cannot find reference object {} in attached_collision_object!".format(attached_collision_object.reference_object_name)) 
        
        for env_object in attached_collision_object.objects:
            tf_ref_object = tf_map_old_reference_object.inv() * SE3(pose_to_list(env_object.origin))
            tf_map_new_object = tf_map_new_ref_object * tf_ref_object
            pose = env_object.origin
            pose.x, pose.y, pose.z, pose.qx, pose.qy, pose.qz, pose.qw = tf_map_new_object.xyz_quat
        
        motion_payload["attached_collision_object"] = attached_collision_object
        to_workspace_id = attached_collision_object.to_workspace_id 
        attached_collision_object.to_workspace_id = 'world'
        if not planning_environment_ros.get_workspace_ros('world'):
            raise Exception("Cannot find 'world' workspace in planning environment!") 
        planning_environment_ros.place_container_items(attached_collision_object, tf_map_new_ref_object)
        attached_collision_object.from_workspace_id = 'world'
        attached_collision_object.to_workspace_id = to_workspace_id
        res = movement_flow.attach_object(attached_collision_object)
        if not res:
            raise Exception("init_movement_flow failed! Please check movement flow log!")
        
    motion_payload["movement_flow"] = movement_flow
    motion_payload["robot_id"] = self.smart_data["0-config-basic"]["robot_id"]
    motion_payload["robot_dof"] = planning_robot_ros.get_kinematics_solver().get_dof()
    gvm.set_variable("motion_payload", motion_payload, per_reference=True)
    
    return "success"
