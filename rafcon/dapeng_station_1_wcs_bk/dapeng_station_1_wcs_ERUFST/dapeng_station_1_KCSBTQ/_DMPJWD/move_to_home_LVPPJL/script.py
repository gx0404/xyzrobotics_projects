import rospy
from xyz_motion import RobotDriver, SmartMoveType
from xyz_motion.rafcon_interface import MovementFlow
from xyz_motion.common import Rn
from xyz_motion.solvers import WoptSolverConfig, AStarConfig, SmartPlannerOptions
from xyz_motion import PlanningEnvironmentRos
from xyz_motion import pose_to_list
from xyz_motion import AllRobotToolStatesRos
from py_xyz_mf import MovementFlow, MoveType, SolverConfig, LogLevel, TrajectorySolverConfig, InterpConfig
from xyz_env_manager.client import get_planning_environment, get_all_robot_tool_states
from xyz_motion import SE3
from rafcon.xyz_exception_base import XYZMotionException
import numpy as np
import copy
import time
from rafcon.xyz_exception_base import XYZExceptionBase

def execute(self, inputs, outputs, gvm):
    """ 
    This state offers a convenient way for user to move to a certain pose without setting up a whole movement flow states and execution states
    Robot is assumed having no item on it and the movement will be executed in full speed in blocked mode.

    Please notice that this state plan and also execute. 
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

    ## TODO input robot state rather than q0
    if inputs["q0"]:
        q0 = inputs["q0"]
    else:
        rob = RobotDriver(int(self.smart_data["0-config-basic"]["robot_id"]))
        ## TODO Let robot gives us a flag of robot is moving.
        q0 = list(rob.get_joints())

    atol = self.smart_data["1-config-interp"]["fine_angle_rez"]
    if np.allclose(q0, np.array(self.smart_data["3-config-node"]["target_joints"]), rtol=0, atol=atol):
        outputs["qf"] = q0
        return "success"

    config = SolverConfig()
    config.timeout = self.smart_data["0-config-basic"]["timeout"]
    config.log_level = getattr(LogLevel, self.smart_data["0-config-basic"]["log_level"])
    config.enable_early_check = self.smart_data["0-config-basic"]["enable_early_check"]
    config.dump_failure = True
    config.keep_conf = False
    config.guess_cache_size = 200
    config.choice_cache_size = 10
    config.dump_failure = self.smart_data["0-config-basic"]["dump_failure"]
    
    interp_config = InterpConfig()
    for key, value in self.smart_data["1-config-interp"].items():
        interp_config.__setattr__(key, value)
    motion_payload["interp_config"] = interp_config

    ## wopt config setting
    wopt_config = WoptSolverConfig()
    wopt_config.dcheck = [self.smart_data["2-config-wopt"]["dcheck"]]
    wopt_config.dsafe = [self.smart_data["2-config-wopt"]["dsafe"]]
    wopt_config.limit_joint = True
    wopt_config.merit_error_coeff = 1.0
    wopt_config.number_of_swept_interpolations = 4
    wopt_config.trust_box_size = 0.5
    wopt_config.do_log = True
    wopt_config.log_directory = "/home/xyz/xyz_log/wopt_log"
    wopt_config.post_solve_simplification = self.smart_data["2-config-wopt"]["simplify"]
    
        
    astar_config = AStarConfig()
    smart_planner_options = SmartPlannerOptions()
    smart_planner_options.do_log = True
    smart_planner_options.log_directory = "/home/xyz/xyz_log/smart_planner_log"
    smart_planner_options.post_solve_simplification = self.smart_data["2-config-wopt"]["simplify"]
    
    solver_config = TrajectorySolverConfig()
    solver_config.wopt_config = wopt_config
    solver_config.astar_config = astar_config
    solver_config.smart_planner_config = smart_planner_options
    solver_config.safety_distance.at_goal = 0.025
    solver_config.safety_distance.at_start = 0.025    
    solver_config.step_num = self.smart_data["2-config-wopt"]["step_num"]
    solver_config.force_vertical = self.smart_data["3-config-node"]["force_vertical"]
    motion_payload["solver_config"] = solver_config
    
    movement_flow = MovementFlow(planning_robot_ros, planning_environment_ros, config)
    ## Add initial node to movement flow
    movement_flow.add_independent_node("initial", 
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
    motion_payload["movement_flow"] = movement_flow
    motion_payload["robot_id"] = self.smart_data["0-config-basic"]["robot_id"]
    motion_payload["robot_dof"] = planning_robot_ros.get_kinematics_solver().get_dof()

    target_joints = [np.array(self.smart_data["3-config-node"]["target_joints"])]
        
    res = movement_flow.add_independent_node(
        "home",
        "", [],
        target_joints,
        [], 
        False, False, "tail", ## check collision from this node to last node
        np.array([0, 0, 0]),
        [],
        "",
        #MoveType.Wopt,
        MoveType.SmartPlanner,
        motion_payload["interp_config"],
        motion_payload["solver_config"]
    )
    
    if not res:
        raise Exception("Adding absolute poses failed! Please check movement flow log!")
    
    solver_cache = gvm.get_variable("solver_cache", per_reference=True, default=[])
    if not solver_cache:
        # TODO: Read solver cache from file
        pass

    movement_flow.set_solver_cache(solver_cache)

    # solving
    solution = None
    try:
        solution = movement_flow.solve({})
    except Exception as e:
        self.logger.error("Solve movement flow failed, error: {}".format(e))
    
    if not solution:
        raise XYZExceptionBase("50006", error_msg="机器人回Home点失败,需要手摇机器人在Home点附近。")

    gvm.set_variable("solver_cache", movement_flow.get_solver_cache(), per_reference=True)

    ## update output trajectory
    execution_payload = {}
    execution_payload["solution"] = solution
    execution_payload["robot_id"] = motion_payload["robot_id"]
    execution_payload["grasp_plan"] = motion_payload.get("grasp_plan", None)
    execution_payload["executed_node_index"] = 0
    outputs["execution_payload"] = execution_payload
    
    ## update the end point qf
    node_name, move_type, choice, traj = solution.trajectory[-1]
    outputs["qf"] = traj.T[-1].tolist()

    robot_driver = RobotDriver(int(execution_payload["robot_id"]))

    solution = execution_payload["solution"]
    trajectory = solution.trajectory
    tf_endflange_tip = SE3()
    
    zone = self.smart_data["4-config-execute"]["zone"]
    mid_zone = self.smart_data["4-config-execute"]["mid_zone"]
    node_index = 1
    node_zone = zone
    name, move_type, choice, node_traj =  trajectory[node_index]

    groupDO_value = 0 # group digital output value. It is only used for moveL_GO and moveJ_GO
    if move_type == move_type.MoveL:
        driver_type = SmartMoveType.LINEAR_MOVE
    elif move_type == move_type.MoveJ:
        driver_type = SmartMoveType.JOINT_MOVE
    elif move_type == move_type.Wopt or move_type == move_type.AStar or move_type == move_type.SmartPlanner:
        driver_type = SmartMoveType.TRAJ_MOVE

    robot_driver.smart_move(node_traj.transpose(), 
                    tf_endflange_tip,
                    self.smart_data["4-config-execute"]["speed"], 
                    self.smart_data["4-config-execute"]["acc"],
                    node_zone, 
                    driver_type,
                    mid_zone, groupDO_value
                )
    self.logger.info("executing node: {} using {} ".format(name, move_type))        
    outputs["qf"] = trajectory[1][-1][:,-1].tolist()
    
    return "success"
