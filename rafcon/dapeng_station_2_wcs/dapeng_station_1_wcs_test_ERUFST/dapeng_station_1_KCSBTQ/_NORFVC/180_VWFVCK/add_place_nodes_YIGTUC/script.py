import numpy as np
from cv2 import transform
from xyz_motion import SE3
from xyz_motion import PlanningEnvironmentRos
from xyz_motion.solvers import WoptSolverConfig, AStarConfig
from py_xyz_mf import MoveType, TrajectorySolverConfig, InterpConfig
from rafcon.xyz_exception_base import XYZMotionException


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


def execute(self, inputs, outputs, gvm):
    """ 
    User can use this state to assign a waypoint in motion planner by defining where the robot flange should go in joint or cartisian poses.

    Args:
        Inputs Data:
            object_poses: Poses of object when placed.

        Outputs Data:
            None

        Gvm: 
            movement_flow, get/set, (motion_planner.rafcon_interface.MovementFlow): 
                Movementflow to manage waypoints, planning environment, and execute motion planning
            motion_payload (dict) (set/get):
                A compund payload to store all necessary information for motion planning, environment updating, and driver execution.
                In this state, information including whether item being picked at this node and how the node being reached is saved in this payload.
            
    Properties Data:
        详见此模块的说明文档.
      


    Raises:
        XYZMotionException: error_code="E0702", raised if movement flow is uninitialized. This happens if the state to initialize movement flow is not added in the state machine
        XYZMotionException: error_code="E0703", raised if length a pose is greater than 7.

    Outcomes:
        0: success, this node is successfully added in the movementflow with valid poses and logic of whether item is on the robot
        -1: aborted
        -2: preempted
    """
    self.logger.info("Running {}({})".format(self.name, self.state_id))
    self.logger.info(self.smart_data["comment"])

    preplace_use = self.smart_data["0-pre_place"]["use"]
    preplace_relative_poses = self.smart_data["0-pre_place"]["relative_poses"]
    pick_conveyor_collision = self.smart_data["0-pre_place"]["pick_conveyor_collision"]
    slidestart_use = self.smart_data["1-slide_start"]["use"]
    slidestart_relative_poses = self.smart_data["1-slide_start"]["relative_poses"]
    nearplace_use = self.smart_data["2-near_place"]["use"]
    nearplace_relative_poses = self.smart_data["2-near_place"]["relative_poses"]
    place_object_poses = inputs["object_poses"]
    postplace_use = self.smart_data["4-post_place"]["use"]
    postplace_relative_poses = self.smart_data["4-post_place"]["relative_poses"]   
    inverse_pre_place = self.smart_data["4-post_place"]["inverse_pre_place"]
    grasp_plan = inputs["grasp_plan"]
    motion_payload = gvm.get_variable("motion_payload", per_reference=True, default=None)
    if not motion_payload:
        raise Exception("必须先初始化运动规划！")

    def get_place_drop_buffer(planning_environment, grasp_plan):
        unfinished_items = planning_environment.get_unfinished_planned_items(str(grasp_plan.to_workspace_id))
        planned_item_id = grasp_plan.planned_items_ids[0]
        for item in unfinished_items:
            if planned_item_id == item.name:
                planned_item = item
                break
        if "place_drop_buffer" in planned_item.additional_info.keys:
            return float(dict(zip(planned_item.additional_info.keys, planned_item.additional_info.values))["place_drop_buffer"])
        return self.smart_data["3-place"]["place_drop_buffer"]

    place_drop_buffer = get_place_drop_buffer(PlanningEnvironmentRos.from_ros_msg(motion_payload["planning_environment"]), motion_payload["grasp_plan"])
    
    movement_flow = motion_payload["movement_flow"]
    
    # pre_place
    if preplace_use:
        if slidestart_use:
            relative_node_name = "slide_start"
        elif nearplace_use:
            relative_node_name = "near_place"
        else:
            relative_node_name = "place"

        group_idx = []
        if preplace_relative_poses and type(preplace_relative_poses[-1]) == tuple:
            group_idx = list(preplace_relative_poses[-1])
            relative_poses = [SE3(normalized_xyz_quat(pose)) for pose in preplace_relative_poses[:-1]]
        else:
            relative_poses = [SE3(normalized_xyz_quat(pose)) for pose in preplace_relative_poses]
        motion_payload["solver_config"].force_vertical = True 

        res = movement_flow.add_relative_node(
            "pre_place", 
            "${ef}",
            relative_poses, ## relative_poses
            "${to}",
            relative_node_name,
            group_idx,
            False, False, "tail", ## check collision from this node to last node
            np.array([0.0, 0.0, 0.0]),
            [],
            "",
            getattr(MoveType, "SmartPlanner"),
            #MoveType.Wopt,
            motion_payload["interp_config"],
            motion_payload["solver_config"]
        )
        if not res:
            raise Exception("无法正确添加pre_place点，详情请查看运动规划日志!")        
        # consider conveyor collision from post_pick to pre_place
        if pick_conveyor_collision:
            planning_env = movement_flow.get_environment("pre_place")
            if not grasp_plan or planning_env.get_workspace_ros(grasp_plan.from_workspace_id).get_type_string() != "conveyor":
                raise Exception("如果要考虑抓取位输送线的碰撞，请确认抓取位是输送线，并连接抓取方案输入！")
            pick_workspace_id = grasp_plan.from_workspace_id
            planning_env.add_full_padding_workspace(pick_workspace_id)

    # slide_start
    if slidestart_use:
        relative_node_name = "near_place" if nearplace_use else "place"
        group_idx = []
        if slidestart_relative_poses and type(slidestart_relative_poses[-1]) == tuple:
            group_idx = list(slidestart_relative_poses[-1])
            relative_poses = [SE3(normalized_xyz_quat(pose)) for pose in slidestart_relative_poses[:-1]]
        else:
            relative_poses = [SE3(normalized_xyz_quat(pose)) for pose in slidestart_relative_poses]
        motion_payload["solver_config"].force_vertical = False
        res = movement_flow.add_relative_node(
            "slide_start", 
            "${ef}",
            relative_poses, ## relative_poses
            "${to}",
            relative_node_name,
            group_idx,
            False, True, "tail", ## check collision from this node to last node
            np.array([0.0, 0.0, 0.0]),
            [],
            "",
            getattr(MoveType, "MoveL") ,
            motion_payload["interp_config"],
            motion_payload["solver_config"]
        )
        if not res:
            raise Exception("无法正确添加slide_start点，详情请查看运动规划日志！")        
        # pick_place_flag = 0, we do nothing

    # near_place
    if nearplace_use:
        relative_node_name = "place"
        group_idx = []
        if nearplace_relative_poses and type(nearplace_relative_poses[-1]) == tuple:
            group_idx = list(nearplace_relative_poses[-1])
            relative_poses = [SE3(normalized_xyz_quat(pose)) for pose in nearplace_relative_poses[:-1]]
        else:
            relative_poses = [SE3(normalized_xyz_quat(pose)) for pose in nearplace_relative_poses]
        motion_payload["solver_config"].force_vertical = False
        if slidestart_use:
            extended_object_dsafe = np.array(self.smart_data["1-slide_start"]["extended_object_dsafe"])
        else:
            extended_object_dsafe = np.array([0.0, 0.0, 0.0])
        res = movement_flow.add_relative_node(
            "near_place", 
            "${ef}",
            relative_poses, ## relative_poses
            "${to}",
            relative_node_name,
            group_idx,
            True, False, "tail", ## check collision from this node to last node
            extended_object_dsafe,
            [],
            "",
            getattr(MoveType, "MoveL") ,
            motion_payload["interp_config"],
            motion_payload["solver_config"]
        )
        if not res:
            raise Exception("无法正确添加near_place点，详情请查看运动规划日志！")        
        # place_place_flag = 0, we do nothing

    # place
    motion_payload["solver_config"].force_vertical = False
    group_idx = []
    if place_object_poses and type(place_object_poses[-1]) == tuple:
        group_idx = list(place_object_poses[-1])
        object_poses = [SE3(normalized_xyz_quat(pose[0:2] + [pose[2] + place_drop_buffer] + pose[3:])) for pose in place_object_poses[:-1]]
    else:
        object_poses = [SE3(normalized_xyz_quat(pose[0:2] + [pose[2] + place_drop_buffer] + pose[3:])) for pose in place_object_poses]
    object_frame_name = "${obj}"

    if slidestart_use and not nearplace_use:
        cc_path = True
        extended_object_dsafe = np.array(self.smart_data["1-slide_start"]["extended_object_dsafe"])
    else:
        cc_path = False
        extended_object_dsafe = np.array([0.0, 0.0, 0.0])

    res = movement_flow.add_object_node(
        "place", 
        "object/" + object_frame_name,
        object_poses,
        group_idx,
        cc_path, False, "tail", ## check collision from this node to last node
        extended_object_dsafe,
        [],
        "",
        getattr(MoveType, "MoveL") ,
        motion_payload["interp_config"],
        motion_payload["solver_config"]
    )
    if not res:
        raise Exception("无法正确添加place点，详情请查看运动规划日志！")   
    if pick_conveyor_collision:
        planning_env = movement_flow.get_environment("place")
        pick_workspace_id = grasp_plan.from_workspace_id
        planning_env.remove_full_padding_workspace(pick_workspace_id)     
    # pick_place_flag = 2, we do the following
    res = movement_flow.detach_object(SE3([0, 0, -place_drop_buffer, 0, 0, 0, 1]))
    if not res:
        raise Exception("无法在放置时清除吸具上的物体，详情请查看运动规划日志！")


    # post_place
    if postplace_use:
        group_idx = []
        if postplace_relative_poses and type(postplace_relative_poses[-1]) == tuple:
            group_idx = list(postplace_relative_poses[-1])
            relative_poses = [SE3(normalized_xyz_quat(pose)) for pose in postplace_relative_poses[:-1]]
        else:
            relative_poses = [SE3(normalized_xyz_quat(pose)) for pose in postplace_relative_poses]

        if inverse_pre_place:
            self.logger.warning("从place到post_place的轨迹被设置为原路返回到pre_place点")
            relative_node_name = "pre_place"
            relative_poses = [SE3([0, 0, 0, 0, 0, 0, 1])]
        else:
            relative_node_name = "place"

        motion_payload["solver_config"].force_vertical = False
        res = movement_flow.add_relative_node(
            "post_place", 
            "${ef}",
            relative_poses, ## relative_poses
            "${to}",
            relative_node_name,
            group_idx,
            False, False, "tail", ## check collision from this node to last node
            np.array([0.0, 0.0, 0.0]),
            [],
            "",
            getattr(MoveType, "MoveL") ,
            motion_payload["interp_config"],
            motion_payload["solver_config"]
        )
        if not res:
            raise Exception("无法正确添加post_place点，详情请查看运动规划日志！")        
        # pick_place_flag = 0, we do nothing

    # end of place nodes
    gvm.set_variable("motion_payload", motion_payload, per_reference=True)
    return "success"
