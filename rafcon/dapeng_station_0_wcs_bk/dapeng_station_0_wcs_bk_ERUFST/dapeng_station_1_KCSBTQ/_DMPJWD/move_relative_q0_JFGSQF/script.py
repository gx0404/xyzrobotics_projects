import tf.transformations as tfm
import rospy
import time
import tf
import numpy as np
from xyz_motion import SE3
from xyz_motion import RobotDriver
from py_xyz_robot.robot_config import get_robot_name
from xyz_motion import create_kinesolver
from xyz_motion import pose_to_list
from xyz_env_manager.client import get_robot_base_transform

def execute(self, inputs, outputs, gvm):
    """ 
    Execute a robot pose based on an pose. 

    This state depends on the rosnode xyz_robot_node which controls robot.
    (Intall by ``sudo xyz_apt update``, ``sudo xyz_apt install xyz-robot``. Run by ``rosrun xyz_robot xyz_robot_node``.)

    Args:
        Inputs Data:
            relative_q0, (unicode): Default value ([]]).
                The base pose you want to relative.

        Outputs Data:
            None

        Gvm: 
            None
            
    Properties Data:
        comment, (unicode): Default value (u"move").
            The comment of this state. This will show in state's GUI block.

        robot_id, (unicode): Default value (u"0").
            The robot id you want to control.

        acc, (list): Default value ([10, 10]).
            Acceleration and deceleration of robot.

        speed, (list): Default value ([10, 10]).
            Linear and angular velocity of robot.

        current_tip, (list): Default value ([0, 0, 0, 0, 0, 0, 1]).
            current_tip is the tip pose relative to robot endflange pose. This is only take effect when move_type is moveL.
            current_tip is the interpolation point when moveL.

        move_type, (unicode, moveL/moveJ): Default value (moveL).
            Robot move type.

        zone, (int): Default value (0).
            Smoothness of execution pose. 

        relative_frame, (unicode): Default value (u"base_link").
            relative frame is the frame we relative to target robot's robot state.
        
        relative_pose, (list): Default value (u"ID").
            relative_pose is the transformation relative to  relative_state_id's relative_frame.

    Raises:
        Exception: invalid move_type
        Exception: target_joints outside of joints limit
        Exception: move_relative has formed a loop
        Exception: move_relative must relative to move or move_relative

    Outcomes:
        0: success
        -1: aborted
        -2: preempted
    """


    self.logger.info("Running {}({})({})".format(self.name, self.unique_id, self.smart_data["comment"]))

    robot_id = int(str(self.smart_data["robot_id"]))
    kinematics_solver = create_kinesolver(get_robot_name(robot_id))
    tf_base_robot = pose_to_list(get_robot_base_transform(self.smart_data["robot_id"]))
    kinematics_solver.set_basetf(SE3(tf_base_robot))

    rob_driver = RobotDriver(robot_id)
    rob_driver.set_basetransform(*tf_base_robot)

    relative_q0 = inputs["relative_q0"]
    endflange_cartpose_q0 =  kinematics_solver.compute_fk(relative_q0)


    if self.smart_data["relative_frame"] == 'end_flange': 
        new_trans = endflange_cartpose_q0 * SE3(self.smart_data["relative_pose"]) * endflange_cartpose_q0.inv()
    elif self.smart_data["relative_frame"] in ['world', 'base_link']:  
        world = SE3(np.eye(3))      
        new_trans = world * SE3(self.smart_data["relative_pose"]) * world.inv()
    
    final_endflange_cartpose = new_trans * endflange_cartpose_q0
    target_joints = kinematics_solver.compute_best_ik(final_endflange_cartpose, relative_q0)
    

    if len(target_joints) == 0:
        raise Exception("Target_joints is unreachable")

    lower, upper = kinematics_solver.get_joint_limit()

    if len(target_joints) != len(lower):
        raise Exception("The length of target_joints must equals to the length of joints_limit")

    for index in range(len(target_joints)):
        if lower[index] > target_joints[index] or target_joints[index] > upper[index]:
            raise Exception("target_joint index: {} is not inside the joint limit, lower: {}, upper: {}".format(index, lower[index], upper[index]))

    if str(self.smart_data["move_type"]) == "moveJ":
        move_type = 1 ## SetJointsMoveJ
    elif str(self.smart_data["move_type"]) == "moveL":
        move_type = 0 ## SetJointsMoveL
    else:
        raise Exception("move type is must moveL/moveJ")

    rob_driver.move(joints = target_joints,
                    tool = SE3(self.smart_data["current_tip"]),
                    speed = self.smart_data["speed"],
                    acc = self.smart_data["acc"],
                    zone = self.smart_data["zone"],
                    move_type = move_type)
    

    outputs["qf"] = target_joints

    return "success"