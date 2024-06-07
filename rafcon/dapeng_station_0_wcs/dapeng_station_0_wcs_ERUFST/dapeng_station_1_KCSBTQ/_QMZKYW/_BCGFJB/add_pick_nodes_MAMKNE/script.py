import numpy as np
from cv2 import transform
from xyz_motion import SE3
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
    User can use this state to set up robot poses at nodes related to pick-node in a standard motion planner.

    Args:
        Inputs Data:
            frame_poses: 
                Endflange poses at pick node.

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

    prepick_use = self.smart_data["0-pre_pick"]["use"]
    prepick_relative_poses = self.smart_data["0-pre_pick"]["relative_poses"]
    place_conveyor_collision = self.smart_data["0-pre_pick"]["place_conveyor_collision"]
    nearprepick_use = self.smart_data["1-near_pre_pick"]["use"]
    nearprepick_relative_poses = self.smart_data["1-near_pre_pick"]["relative_poses"]
    pick_frame_poses = inputs["frame_poses"]
    nearpostpick_use = self.smart_data["2-near_post_pick"]["use"]
    nearpostpick_relative_poses = self.smart_data["2-near_post_pick"]["relative_poses"]
    postpick_use = self.smart_data["3-post_pick"]["use"]
    postpick_relative_poses = self.smart_data["3-post_pick"]["relative_poses"]   
    grasp_plan = inputs["grasp_plan"]
    motion_payload = gvm.get_variable("motion_payload", per_reference=True, default=None)
    if not motion_payload:
        raise Exception("必须先初始化运动规划！")

    movement_flow = motion_payload["movement_flow"]
    
    # pre_pick
    if prepick_use:
        relative_node_name = "near_pre_pick" if nearprepick_use else "pick"
        group_idx = []
        if prepick_relative_poses and type(prepick_relative_poses[-1]) == tuple:
            group_idx = list(prepick_relative_poses[-1])
            relative_poses = [SE3(normalized_xyz_quat(pose)) for pose in prepick_relative_poses[:-1]]
        else:
            relative_poses = [SE3(normalized_xyz_quat(pose)) for pose in prepick_relative_poses]
        motion_payload["solver_config"].force_vertical = True 
        res = movement_flow.add_relative_node(
            "pre_pick", 
            "${ef}",
            relative_poses, ## relative_poses
            "${from}",
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
            raise Exception("无法正确添加pre_pick点，详情请查看运动规划日志！")        
        # consider conveyor collision from post_place to pre_pick
        if place_conveyor_collision:
            planning_env = movement_flow.get_environment("pre_pick")
            if not grasp_plan or planning_env.get_workspace_ros(grasp_plan.to_workspace_id).get_type_string() != "conveyor":
                raise Exception("如果要考虑放置位输送线的碰撞，请确认放置位是输送线，并连接抓取方案输入！")
            place_workspace_id = grasp_plan.to_workspace_id
            planning_env.add_full_padding_workspace(place_workspace_id)

    # near_pre_pick
    if nearprepick_use:
        relative_node_name = "pick"
        group_idx = []
        if nearprepick_relative_poses and type(nearprepick_relative_poses[-1]) == tuple:
            group_idx = list(nearprepick_relative_poses[-1])
            relative_poses = [SE3(normalized_xyz_quat(pose)) for pose in nearprepick_relative_poses[:-1]]
        else:
            relative_poses = [SE3(normalized_xyz_quat(pose)) for pose in nearprepick_relative_poses]
        motion_payload["solver_config"].force_vertical = False
        res = movement_flow.add_relative_node(
            "near_pre_pick", 
            "${ef}",
            relative_poses, ## relative_poses
            "${from}",
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
            raise Exception("无法正确添加near_pre_pick点，详情请查看运动规划日志！")        
        # pick_place_flag = 0, we do nothing

    # pick
    motion_payload["solver_config"].force_vertical = False
    target_joints = []
    group_idx = []
    if pick_frame_poses and type(pick_frame_poses[-1]) == tuple:
        group_idx = list(pick_frame_poses[-1])
        frame_poses = [SE3(normalized_xyz_quat(pose)) for pose in pick_frame_poses[:-1]]
    else:
        frame_poses = [SE3(normalized_xyz_quat(pose)) for pose in pick_frame_poses]
    res = movement_flow.add_independent_node(
        "pick", 
        "${ef}", frame_poses,
        target_joints,
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
        raise Exception("无法正确添加pick点，详情请查看运动规划日志！")        
    if place_conveyor_collision:
        planning_env = movement_flow.get_environment("pick")
        planning_env.remove_full_padding_workspace(place_workspace_id)
    # pick_place_flag = 1, we do the following
    if not motion_payload.get("attached_collision_object", None):
        raise Exception("抓取点没有可选的抓取物体！")
    attached_collision_object = motion_payload["attached_collision_object"]
    res = movement_flow.attach_object(attached_collision_object)
    if not res:
        raise Exception("无法正确添加pick点，详情请查看运动规划日志！")


    # near_post_pick
    if nearpostpick_use:
        relative_node_name = "pick"
        group_idx = []
        if nearpostpick_relative_poses and type(nearpostpick_relative_poses[-1]) == tuple:
            group_idx = list(nearpostpick_relative_poses[-1])
            relative_poses = [SE3(normalized_xyz_quat(pose)) for pose in nearpostpick_relative_poses[:-1]]
        else:
            relative_poses = [SE3(normalized_xyz_quat(pose)) for pose in nearpostpick_relative_poses]
        motion_payload["solver_config"].force_vertical = False
        res = movement_flow.add_relative_node(
            "near_post_pick", 
            "${ef}",
            relative_poses, ## relative_poses
            "${from}",
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
            raise Exception("无法正确添加near_post_pick点，详情请查看运动规划日志！")        
        # pick_place_flag = 0, we do nothing

    # post_pick
    if postpick_use:
        relative_node_name = "near_post_pick" if nearpostpick_use else "pick"
        group_idx = []
        if postpick_relative_poses and type(postpick_relative_poses[-1]) == tuple:
            group_idx = list(postpick_relative_poses[-1])
            relative_poses = [SE3(normalized_xyz_quat(pose)) for pose in postpick_relative_poses[:-1]]
        else:
            relative_poses = [SE3(normalized_xyz_quat(pose)) for pose in postpick_relative_poses]
        motion_payload["solver_config"].force_vertical = False
        res = movement_flow.add_relative_node(
            "post_pick", 
            "${ef}",
            relative_poses, ## relative_poses
            "${from}",
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
            raise Exception("无法正确添加post_pick点，详情请查看运动规划日志！")        
        # pick_place_flag = 0, we do nothing

    # end of pick nodes
    gvm.set_variable("motion_payload", motion_payload, per_reference=True)
    return "success"
