from cv2 import transform
from xyz_motion import SE3
from xyz_motion.solvers import WoptSolverConfig, AStarConfig
from py_xyz_mf import MoveType, TrajectorySolverConfig, InterpConfig
from rafcon.xyz_exception_base import XYZMotionException
import numpy as np

def execute(self, inputs, outputs, gvm):
    """ 
    User can use this state to assign a waypoint in motion planner by defining where the robot flange should go in joint or cartisian poses.

    Args:
        Inputs Data:
            None

        Outputs Data:
            None

        Gvm: 
            movement_flow, get/set, (motion_planner.rafcon_interface.MovementFlow): 
                Movementflow to manage waypoints, planning environment, and execute motion planning
            motion_payload (dict) (set/get):
                A compund payload to store all necessary information for motion planning, environment updating, and driver execution.
                In this state, information including whether item being picked at this node and how the node being reached is saved in this payload.
            
    Properties Data:
        check_collision_path, (bool): False.
            Whether the path between this waypoint and the next should perform collision checking
        check_collsion_point, (bool): False.
            Whether this waypoint should be checked collision. This overwirtes the setting in check_collision_path
        comment, (unicode): add_relative_poses.
            Recommend to assign a easy to recognize waypoint name in this field.
        move_type (unicode): "moveL"
             which movement mode needs to be executed to reach to current waypoint. Can be "moveL", "moveJ", "moveL_wopt", or "moveJ_wopt". The suffix "wopt" only determines wopt will be used for motion planning.
        node_name (unicode): None
            Recommend to assign a easy to recognize waypoint name in this field.
        pick_place_flag (int): 0.
            An int value determines the item status relative to the robot. 
            0: Robot did not change the status whether it has item on or not. 
            1: Robot picks up item at this moment. 
            2: Robot drop item at this moment. 
            3: Robot pick or still have item on but will ignore it in collision checking.   
            In one motion task, robot can only pick or drop item once.

        static_endflange_poses (list): [].
            A list of poses which each one can be a quaternion cartisian pose or joint pose. 
            It will be overwritted if input data has provided these poses.

        target_frame (unicode): "${ef}"
            The target frame to move relatively.
        base_frame (unicode): "${robot}":
            In which frame the relative motion is observed.
        relative_node_name (unicode): None.
            The waypoint that is related to for the current motion. 
        fixed_DoFs(dict): a dict of which position or orientation should be fixed. A full dict is {"x": ,"y": ,"z": ,"qx": ,"qy": ,"qz": ,"qw":}
        relative_poses (list): [[]].
            Each member is a 7-length transformation with orientation in quaternion


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
    
    motion_payload = gvm.get_variable("motion_payload", per_reference=True, default=None)
    if not motion_payload:
        raise Exception("state init_movement_flow needs to be activated before this one!")

    movement_flow = motion_payload["movement_flow"]
    
    motion_payload["solver_config"].force_vertical = self.smart_data["force_vertical"]

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

    group_idx = []
    if self.smart_data["relative_poses"] and type(self.smart_data["relative_poses"][-1]) == tuple:
        group_idx = list(self.smart_data["relative_poses"][-1])
        relative_poses = [SE3(normalized_xyz_quat(pose)) for pose in self.smart_data["relative_poses"][:-1]]
    else:
        relative_poses = [SE3(normalized_xyz_quat(pose)) for pose in self.smart_data["relative_poses"]]

    if self.smart_data["place_drop_buffer"] > 1e-5:
        raise Exception("Relative pose node does not support place_drop_buffer!")
    if np.any(self.smart_data["extended_object_dsafe"]) and not self.smart_data["check_collision_path"]:
        self.logger.warning("extended_object_dsafe takes no effect, because check_collision_path is set to false. Please check your settings.")

    res = movement_flow.add_relative_node(
        str(self.smart_data["node_name"]), 
        self.smart_data["target_frame"],
        relative_poses, ## relative_poses
        self.smart_data["base_frame"],
        self.smart_data["relative_node_name"],
        group_idx,
        self.smart_data["check_collision_path"], self.smart_data["check_collsion_point"], "tail", ## check collision from this node to last node
        np.array(self.smart_data["extended_object_dsafe"]),
        self.smart_data["ignored_objects"],
        self.smart_data["activated_mesh_name"],
        getattr(MoveType, self.smart_data["move_type"]) ,
        motion_payload["interp_config"],
        motion_payload["solver_config"]
    )

    if not res:
        raise Exception("Adding relative poses failed! Please check movement flow log!")
            
    # TODO: Sanity check in cpp
    pick_place_flag = self.smart_data["pick_place_flag"]
    
    if pick_place_flag == 0: ## same as last node
        pass
    elif pick_place_flag == 1: ## pick item at this node
        if not motion_payload.get("attached_collision_object", None):
            raise Exception("pick_place_flag cannot be non-zero when grasp_plan is empty!")
        attached_collision_object = motion_payload["attached_collision_object"]
        res = movement_flow.attach_object(attached_collision_object)
        if not res:
            raise Exception("attach_object failed when adding absolute poses! Please check movement flow log!")
    elif pick_place_flag == 2:
        res = movement_flow.detach_object(SE3([0, 0, -self.smart_data["place_drop_buffer"], 0, 0, 0, 1]))
        if not res:
            raise Exception("detach_object failed when adding absolute poses! Please check movement flow log!")
    else:
        raise Exception("pick_place_flag value: {} is not supported".format(pick_place_flag))

    gvm.set_variable("motion_payload", motion_payload, per_reference=True)
    return "success"