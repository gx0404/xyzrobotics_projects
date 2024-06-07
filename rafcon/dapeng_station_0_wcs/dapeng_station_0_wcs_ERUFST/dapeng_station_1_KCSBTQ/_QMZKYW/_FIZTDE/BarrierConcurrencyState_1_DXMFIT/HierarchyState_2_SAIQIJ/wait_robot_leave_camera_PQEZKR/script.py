import rospy
import time

from xyz_motion import RobotRos
from xyz_motion import CollisionChecker
from xyz_motion import PlanningEnvironmentRos
from xyz_motion import AllRobotToolStatesRos
from xyz_env_manager.msg import PrimitiveGroup
from xyz_env_manager.msg import GeometricPrimitive
from xyz_env_manager.msg import Pose
from xyz_motion import RobotDriver
from xyz_env_manager.client import get_all_robot_tool_states
from xyz_env_manager.client import get_planning_environment

def check_robot_within_ws(robotdriver, collisionChecker):
    q0 = robotdriver.get_joints()
    all_robot_ros = get_all_robot_tool_states()
    our_robot = AllRobotToolStatesRos.from_ros_msg(all_robot_ros).get_robot_ros('0')
    kinematic_solver = our_robot.get_kinematics_solver()
    if len(q0)==4:
        q0 = kinematic_solver.convert_four_dof_to_six(q0)
    return collisionChecker.check_point_collision(q0)

def execute(self, inputs, outputs, gvm):

    self.logger.info("Running {}({})({})".format(self.name, self.unique_id, self.smart_data["comment"]))

    pl_msg = get_planning_environment()
    planning_env = PlanningEnvironmentRos.from_ros_msg(pl_msg)

    workspace_id = self.smart_data["workspace_id"]
    dim_scale = self.smart_data["dim_scale"]
    assert (dim_scale["X"]>=1.0 and dim_scale["Y"]>=1.0)
    if dim_scale["Z"] <=10.0:
        self.logger.warning("考虑碰撞的相机视野区域高度过小， 可能造成机械臂还在视野中就拍照！")

    workspace_msg = planning_env.get_workspace_ros(workspace_id).to_ros_msg()
    
    pg = PrimitiveGroup(name="camera"+ str(workspace_id))
    pg.origin = workspace_msg.bottom_pose
    pg.primitives.append(
            GeometricPrimitive(type=GeometricPrimitive.BOX, 
                                dimensions=[workspace_msg.dimensions[0] * dim_scale["X"], 
                                            workspace_msg.dimensions[1] * dim_scale["Y"], 
                                            workspace_msg.dimensions[2] * dim_scale["Z"]], 
                                relative_pose=Pose(0, 0, workspace_msg.dimensions[2] * dim_scale["Z"]/2, 0, 0, 0, 1)))
    res = planning_env.add_new_primitive_group(pg)
    if not res.success:
        raise Exception("障碍物名字重复，工作空间障碍物<camera{}>添加失败".format(str(workspace_id)))
        
    # setup robot model
    all_robot_ros = get_all_robot_tool_states()
    robot_id = self.smart_data["robot_id"]
    our_robot = AllRobotToolStatesRos.from_ros_msg(all_robot_ros).get_robot_ros(robot_id)
    our_robot.detach_object()
    # Set up collision checker
    checker = CollisionChecker(our_robot, planning_env)
    
    ## start to check collision
    rob = RobotDriver(int(robot_id))
    wait_start_time = time.time()
    time_limit_seconds = self.smart_data["timeout"]

    while not self.preempted:
        #if gvm.variable_exist("ERROR"):
            #return "timeout"
        if time.time() - wait_start_time > time_limit_seconds:
            return "timeout"
        
        if not check_robot_within_ws(rob, checker): 
            break
       
        self.preemptive_wait(0.2)

    return "success"