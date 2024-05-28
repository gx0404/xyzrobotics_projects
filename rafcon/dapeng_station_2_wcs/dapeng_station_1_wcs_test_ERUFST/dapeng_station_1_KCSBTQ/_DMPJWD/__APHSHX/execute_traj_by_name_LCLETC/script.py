from xyz_motion import SE3
from rafcon.xyz_exception_base import XYZMotionException, XYZExceptionBase
from xyz_motion import RobotDriver, SmartMoveType
from xyz_env_manager.msg import Pose
from xyz_env_manager.client import get_all_robot_tool_states, pick_container_items, place_container_items, append_attached_collision_object, clear_attached_collision_object, update_finished_planned_items
from xyz_motion import AllRobotToolStatesRos
from rafcon.utils.helper_function import find_the_nearest_state_in_concurrency_state

def execute(self, inputs, outputs, gvm):
    """ 
    This state execute motion by the target waypoint assigned by the user. The execution_payload to reach the target waypoint is calculated by motion planner.

    This state also has the memory of which waypoints are traversed.
    All motions between the current position and the target waypoint will be executed in a unblocked way.

    Args:
        Inputs Data:
            execution_payload (dict): None
                The motion payload records a verboise information about waypoints, items poses at each waypoint, execution_payload of waypoints, and ways to execute the movements.

        Outputs Data:
            item_info (xyz_env_manager.msg._PrimitiveGroup.PrimitiveGroup): None
                A envrionment manager message represents the item pose in world frame after this state is executed.
                If item is not on the robot after this motion, then this output will be None as item pose will not be tracked in following motions in this cycle.
            qf (list):
                Robot joint pose after the execution of this state
            execution_payload (dict):
                same payload as the input with only "executed_name" has waypoints traversed in this state being appended.


        Gvm:
            speed_scale (float) {get}: 1.0
                A float between 0~1.0 to scale speed for this movement
            acc_scale (float) {get}: 1.0
                A float between 0~1.0 to scale speed for this speed
            

    Properties Data:
        acc, (list): [10, 10]
            acceleration setting motion. It is a length 2 list of float between 0~100 which the former value is for linear motion and the later for joint motion
            each is a int between 0 ~ 100. Stream motion only uses the first digit
        speed, (list): [10, 10]
            speed setting for motion. It is a length 2 list of float between 0~100 which the former value is for linear motion and the later for joint motion
            each is a int between 0 ~ 100. Stream motion only uses the first digit
        blocked (bool):
            Set stream motion is blocked or not
        comment (unicode): execute_traj_by_name
            recommend to assign a name that indicate which target waypoint is goint to be reached at the end of this state
        robot_id (unicode): "0"
            The id of robot for this movement.
        target_name (unicode): None
            The target waypoint name to be reached at the end of execution.
        update_env (bool): True
            Automatically update the environment in this state.
        zone (int): 0
            If zone is 0 and motion is moveL or moveJ, then the last waypoint will be reached with driver running in blocked mode. If zone is greater than 0, then all motions in this state will be executed unblocked.
            Smoothing between waypoints will be greater if zone gets larger. The executed execution_payload might be further away from what is calculated from motion planner and might have higher risk of collision.
        wopt_mid_zone (int): 25
            When using moveJ_wopt, wopt will interpolate between start_pose to goal_pose. THis is the zone of mid-point of execution_payload.(No include the final pose).

    Raises:
        XYZMotionException: error_code="E0704", raised if the target waypoint is already executed in previous states
        XYZMotionException: error_code="E0705", raised if the target waypoint is not defined in movement flow states

    Outcomes:
        0: "success", execution command successfully sent in unblock mode or motion is finished in blocked mode
        -1: aborted
        -2: preempted
    """
    
    self.logger.info("Running {}({})".format(self.name, self.state_id))
    self.logger.info(self.smart_data["comment"])
    

    execution_payload = inputs["execution_payload"]
    robot_driver = RobotDriver(int(execution_payload["robot_id"]))

    solution = execution_payload["solution"]
    trajectory = solution.trajectory
    target_name = self.smart_data["target_name"]   
    executed_node_index =  execution_payload["executed_node_index"]
    ## check if execute this node is valid.
    for node_index in range(executed_node_index + 1, len(trajectory)):
        if target_name == trajectory[node_index][0]:
            target_node_index = node_index
            break
    else:
        raise XYZMotionException(error_code="E0704", error_msg="node {} is not found in execution!".format(target_name))
    

    ## TODO from gvm
    all_robot_states = AllRobotToolStatesRos.from_ros_msg(get_all_robot_tool_states())
    planning_robot_ros = all_robot_states.get_robot_ros(execution_payload["robot_id"])
    grasp_plan = execution_payload.get("grasp_plan", None)
    if not grasp_plan:
        tf_endflange_tip = SE3()
    else:
        tf_endflange_tip = planning_robot_ros.get_active_tool().get_tf_endflange_tip(grasp_plan.tip.name)
    
    zone = self.smart_data["zone"]
    mid_zone = self.smart_data["mid_zone"]
    speed_scale = gvm.get_variable("speed_scale", per_reference=True, default=1)
    acc_scale = gvm.get_variable("acc_scale", per_reference=True, default=1)
    self.logger.info("scale of speed and acc {}  {}".format(speed_scale, acc_scale))
    adjust_speed = [max(1, int(self.smart_data["speed"][0] * speed_scale)), max(1, int(self.smart_data["speed"][1] * speed_scale))]
    adjust_acc = [max(1, int(self.smart_data["acc"][0] * acc_scale)), max(1, int(self.smart_data["acc"][1] * acc_scale))]
    for node_index in range(executed_node_index + 1, target_node_index + 1):
        node_zone = zone if node_index == target_node_index else mid_zone
        if node_index in [solution.grasp_info[0].pick_index, solution.grasp_info[0].place_index] and node_zone != 0:
            self.logger.warning("zone for pick or place node is not 0, the robot is unblocked!")
        
        name, move_type, choice, node_traj =  trajectory[node_index]

        groupDO_value = 0 # group digital output value. It is only used for moveL_GO and moveJ_GO
        if move_type == move_type.MoveL:
            driver_type = SmartMoveType.LINEAR_MOVE
        elif move_type == move_type.MoveJ:
            driver_type = SmartMoveType.JOINT_MOVE
        elif move_type == move_type.Wopt or move_type == move_type.AStar or move_type == move_type.SmartPlanner:
            driver_type = SmartMoveType.TRAJ_MOVE

        # Only support ABB robot for moveL_GO and moveJ_GO for now
        if self.smart_data["allow_movedo"]["enable"] and robot_driver.robot_type == "abb":
            if driver_type == SmartMoveType.JOINT_MOVE:
                driver_type = SmartMoveType.JOINT_MOVE_GO
            elif driver_type == SmartMoveType.LINEAR_MOVE:
                driver_type = SmartMoveType.LINEAR_MOVE_GO
            for i in range(len(self.smart_data["allow_movedo"]["port_id"])):
                groupDO_value = groupDO_value | ( self.smart_data["allow_movedo"]["value"][i] 
                               << self.smart_data["allow_movedo"]["port_id"][i])
        robot_driver.smart_move(node_traj.transpose(), 
                        tf_endflange_tip,
                        adjust_speed, 
                        adjust_acc,
                        node_zone, 
                        driver_type,
                        mid_zone, groupDO_value
                    )
        self.logger.info("executing node: {} using {} ".format(name, move_type))
        
        if node_index == solution.grasp_info[0].pick_index:
            pick_container_items(execution_payload["attached_collision_object"])
            append_attached_collision_object(execution_payload["robot_id"], execution_payload["attached_collision_object"])
        if node_index == solution.grasp_info[0].place_index:
            place_container_items(execution_payload["attached_collision_object"], Pose(*solution.grasp_info[0].tf_map_placed.xyz_quat))
            if solution.solved_env.get_workspace_ros(grasp_plan.to_workspace_id).get_type_string() == "pallet":
                update_finished_planned_items(grasp_plan.to_workspace_id, grasp_plan.planned_items_ids)
            else:
                from xyz_env_manager.client import get_unfinished_planned_items,add_container_items
                from xyz_motion import FormattedRealBox,pose_to_list
                import time
                unfinished_items = get_unfinished_planned_items(grasp_plan.to_workspace_id) 
                sku_info = inputs["sku_info"]
                def build_primitive(item,sku_info):
                    id = int(item.name.rsplit("-")[1])
                    dimensions = list(item.primitives[0].dimensions)
                    timestamp = time.time()
                    item.origin.z -=0.035
                    vision_box_proxy = FormattedRealBox(id = id, 
                                                tf_world_origin = SE3(pose_to_list(item.origin)), 
                                                size = dimensions,
                                                tf_origin_box_center = SE3([0, 0, -dimensions[-1]/2, 0, 0, 0, 1]))
                    pg = vision_box_proxy.primitive_group()

                    # We only support box object currently
                    pg.additional_info.type = "box"
                    pg.additional_info.keys.extend(list(map(str, sku_info.keys())) + ["timestamp"])
                    pg.additional_info.values.extend(list(map(str, sku_info.values())) + [str(timestamp)])
                    pg.additional_info.descriptions = pg.additional_info.keys

                    return pg
                #import ipdb;ipdb.set_trace() 
                containner_item = build_primitive(unfinished_items[0],sku_info)
                add_container_items(grasp_plan.to_workspace_id,[containner_item])
            clear_attached_collision_object(execution_payload["robot_id"], clear_all=True)
        
    execution_payload["executed_node_index"] = target_node_index
    
    outputs["execution_payload"] = execution_payload
    outputs["qf"] = trajectory[target_node_index][-1][:,-1].tolist()
    return "success"
