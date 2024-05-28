import os
import rospy
import tf
import traceback
import sys
from xyz_motion import RobotDriver
from rafcon.utils.helper_function import find_root_state_recursively
from rafcon.xyz_exception_base import XYZExceptionBase

def check_pid_is_valid(pid):        
    current_pid = os.getpid()
    if (current_pid == pid):
        return True
    try:
        ## https://stackoverflow.com/questions/568271/how-to-check-if-there-exists-a-process-with-a-given-pid-in-python
        ## Check For the existence of a unix pid.
        os.kill(pid, 0)
    except OSError:
        return True
    else:
        return False


def execute(self, inputs, outputs, gvm):
    """
    Init Ros.
    Set robot to init joints when in simulation.

    Args:
        Inputs Data:
            None

        Outputs Data:
            None

        Gvm:
            None

    Properties Data:
        comment, (unicode): Default value (u"init_all").
            The comment of this state. This will show in state's GUI block.

        robot_id, (unicode): Default value (u"0").
            Current robot id.

        simulation_start_joints, (list): Default value ([0, 0, 0, 0, 0, 1.57]).
            simulation_start_joints when robot in simulation.


    Raises:
        Exception: rosnode can not start.

    Outcomes:
        0: success
        -1: aborted
        -2: preempted
    """

    self.logger.info("Running {}({})({})".format(self.name, self.unique_id, self.smart_data["comment"]))

    # check if the roscore is already running
    try:
        rospy.wait_for_service("/rosout/get_loggers", 5.0)
    except Exception as e:
        raise Exception("Exception: {} {}".format(e, traceback.format_exc()))


    self.logger.info(sys.version)

    ## namespaces are not allowed in ros node names
    node_name = find_root_state_recursively(self).parent.file_system_path.replace("/", "-")

    try:
        if rospy.get_param(node_name, False) and not check_pid_is_valid(rospy.get_param(node_name)):
            raise Exception("The same graph can only run once")
        rospy.set_param(node_name, os.getpid())
        if not rospy.get_node_uri():
            rospy.init_node(node_name, anonymous=False, disable_signals=True)
            self.logger.info("Creating node: " + node_name)
    except Exception as e:
        raise Exception("Exception: {} {}".format(e, traceback.format_exc()))


    rob_driver = RobotDriver(int(self.smart_data["robot_id"]))

    try:
        rospy.wait_for_service("/robot_{}_GetJoints".format(self.smart_data["robot_id"]), 2.0)
    except Exception as e:
        raise XYZExceptionBase("50008", "1）查看机器人是否断连；2）查看机器人节点是否打开；3）重启机器人节点解除异常")

    # set simulation pose
    if rob_driver.get_robotstatus()["simulation"]:
        rob_driver.set_joints_movej(self.smart_data["simulation_start_joints"])

    from py_xyz_robot import Robot
    r = Robot(0)
    r.set_zone(100)
    
    return "success"
