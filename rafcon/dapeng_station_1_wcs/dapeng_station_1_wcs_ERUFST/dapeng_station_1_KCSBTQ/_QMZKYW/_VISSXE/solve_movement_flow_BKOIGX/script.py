from xyz_motion import AllRobotToolStatesRos
import copy
from rafcon.xyz_exception_base import XYZMotionException, XYZExceptionBase
import copy

def execute(self, inputs, outputs, gvm):
    """
    This state activates the solver for movement flow and produce a motion payload for driver to execute if the motion planner has a solution.
    This state should be connected after all waypoints are added and before execution.

    Args:
        Inputs Data:
            None

        Outputs Data:
            qf (list):
                The joint pose of where the robot ends up after this task
            execution_payload (dict):
                A verbose payload for visualization and driver to execute motion.

        Gvm:
            movement_flow, get/set, (motion_planner.rafcon_interface.MovementFlow):
                Movementflow to manage waypoints, planning environment, and execute motion planning.
                Will be cleared up when this state finished.
            motion_payload (dict) (set/get):
                A compund payload to store all necessary information for motion planning, environment updating, and driver execution.
                In this state, information about how to execute the driver to finish this round of task is produced.
                The information of whether the item ends up in the world frame is also calculated and stored in it.

    Properties Data:
        comment, (unicode): "solve_movement_flow"
            recommend to assign the name of the current task in this field.

        flange_sort (bool): True
            False: No optimization performed when solving paths
            True: Paths with less flange rotation cost will be solved first
    """
    self.logger.info("Running {}({})".format(self.name, self.state_id))
    self.logger.info(self.smart_data["comment"])

    motion_payload = gvm.get_variable("motion_payload", per_reference=True, default=None)
    if not motion_payload:
        raise Exception("state init_movement_flow needs to be activated before this one!")

    solver_cache = gvm.get_variable("solver_cache", per_reference=True, default=[])
    if not solver_cache:
        # TODO: Read solver cache from file
        pass

    movement_flow = motion_payload["movement_flow"]
    movement_flow.set_solver_cache(solver_cache)

    if self.smart_data["flange_sort"]:
        sorter_config = {"rotation": 1.0}
    else:
        sorter_config = {}

    # solving
    solution = None
    try:
        solution = movement_flow.solve(sorter_config)
    except Exception as e:
        self.logger.error("Solve movement flow failed, error: {}".format(e))
    
    if not solution:
        return "failure"

    gvm.set_variable("solver_cache", movement_flow.get_solver_cache(), per_reference=True)

    ## update planning_environment
    if motion_payload.get("grasp_plan", None) and solution.solved_env.get_workspace_ros(motion_payload["grasp_plan"].to_workspace_id).get_type_string() == "pallet":
        solution.solved_env.update_finished_planned_items(motion_payload["grasp_plan"].to_workspace_id, 
                                                            motion_payload["grasp_plan"].planned_items_ids)                                                   
    motion_payload["planning_environment"] = solution.solved_env.to_ros_msg()

    ## update output trajectory
    execution_payload = {}
    execution_payload["solution"] = solution
    execution_payload["robot_id"] = motion_payload["robot_id"]
    attached_collision_object = motion_payload.get("attached_collision_object", None)
    if attached_collision_object:
        a = attached_collision_object.origin_tip_transform
        a.x, a.y, a.z, a.qx, a.qy, a.qz, a.qw = solution.grasp_info[0].tf_tip_object.xyz_quat
        execution_payload["attached_collision_object"] = attached_collision_object
    execution_payload["grasp_plan"] = motion_payload.get("grasp_plan", None)
    execution_payload["executed_node_index"] = 0
    outputs["execution_payload"] = execution_payload
    
    ## update the end point qf
    node_name, move_type, choice, traj = solution.trajectory[-1]
    outputs["qf"] = traj.T[-1].tolist()

    return "success"
