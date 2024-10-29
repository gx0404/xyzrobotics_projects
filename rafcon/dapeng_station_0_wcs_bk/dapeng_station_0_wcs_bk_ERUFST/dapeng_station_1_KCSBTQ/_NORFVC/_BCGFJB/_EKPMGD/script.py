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


def check_robot_within_ws(q0, collisionChecker,kinematic_solver):
    if len(q0)==4:
        q0 = kinematic_solver.convert_four_dof_to_six(q0)
    return collisionChecker.check_point_collision(q0)

def execute(self, inputs, outputs, gvm):
    self.logger.info("Running {}({})({})".format(self.name, self.unique_id, self.smart_data["comment"]))

    pl_msg = get_planning_environment()
    planning_env = PlanningEnvironmentRos.from_ros_msg(pl_msg)

    workspace_ids = self.smart_data["workspace_id_list"]
    dim_scale = self.smart_data["dim_scale"]
    robot_id = self.smart_data["robot_id"]
    rob = RobotDriver(int(robot_id))   

    assert (dim_scale["X"]>=1.0 and dim_scale["Y"]>=1.0)

    check_collision = {}  
    # setup robot model
    all_robot_ros = get_all_robot_tool_states()
    our_robot = AllRobotToolStatesRos.from_ros_msg(all_robot_ros).get_robot_ros(robot_id)
    kinematic_solver = our_robot.get_kinematics_solver()
    current_time = time.time()  
    for _id in workspace_ids:
        workspace_msg = planning_env.get_workspace_ros(_id).to_ros_msg()
        
        pg = PrimitiveGroup(name="robot_test"+ str(_id))
        pg.origin = workspace_msg.bottom_pose
        pg.primitives.append(
                GeometricPrimitive(type=GeometricPrimitive.BOX, 
                                   dimensions=[workspace_msg.dimensions[0] * dim_scale["X"], 
                                               workspace_msg.dimensions[1] * dim_scale["Y"], 
                                               workspace_msg.dimensions[2] * dim_scale["Z"]], 
                                   relative_pose=Pose(0, 0, workspace_msg.dimensions[2] * dim_scale["Z"]/2, 0, 0, 0, 1)))
        res = planning_env.add_new_primitive_group(pg)
        if not res.success:
            raise Exception("障碍物名字重复，工作空间障碍物<robot_test{}>添加失败".format(str(_id)))

        # Set up collision checker
        checker = CollisionChecker(our_robot, planning_env)
        q0 = rob.get_joints()
        if check_robot_within_ws(q0, checker,kinematic_solver):
            self.logger.info(f"检测到机器人在工作空间中间{_id}")
            check_collision[_id] = True
        else:
            self.logger.info(f"工作空间{_id}未检测到机器人")   
            
        res = planning_env.remove_primitive_group("robot_test"+ str(_id))     
        if not res.success:
            raise Exception("无法删除障碍物名字".format(str(_id)))
        self.logger.info(f"time is {time.time()-current_time}")    
            
    if not len(check_collision):
        self.logger.info(f"未检测到机械臂在检测的空间内")
        outputs["other_non_collision_workspace_ids"] = []+["7","8"]
    elif len(check_collision)==1:
        self.logger.info(f"检测到机械臂在空间{list(check_collision.keys())[0]}内")
        #outputs["space_id"] = str(list(check_collision.keys())[0])
        outputs["other_non_collision_workspace_ids"] = [str(list(check_collision.keys())[0])]+["7","8"]
    else: 
        self.logger.info(f"检测到机械臂在多个空间内，或已经碰撞，无法自动回home")
        self.logger.info(f"空间为{check_collision}")               
        raise f"检测到机械臂在多个空间内，或已经碰撞，无法自动回home"
    return "success"