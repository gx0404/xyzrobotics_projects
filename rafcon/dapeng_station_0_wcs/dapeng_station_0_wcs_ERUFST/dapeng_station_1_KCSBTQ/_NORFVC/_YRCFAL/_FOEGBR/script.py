
from xyz_env_manager.msg import Pose
from xyz_env_manager.client import modify_workspace_of_environment,get_planning_environment,modify_primitive_group_of_environment
from xyz_motion import PlanningEnvironmentRos
import tf.transformations as tfm
from xyz_motion import SE3,pose_to_list
from xyz_env_manager.msg import PrimitiveGroup,Pose
from xyz_env_manager.msg import GeometricPrimitive
from xyz_env_manager.client import add_primitive_group_of_environment
import numpy as np

# modify vision result z axis
def modify_z_axis(update_pose, fraction, tf_world_tote):
    """
    this is a funciton to modify the vision result z axis
 
    Args:
        update_pose: original vision result
        fraction: the fraction between [0, 1], 0 means dont modify, and 1 means vision z axis parallel to workspace z axis
        worskspace_id: vision workspace id
    Returns:
        new vision result
    Exception:
 
    """
    quat_old = update_pose[3:]
    tf_world_obj = SE3(update_pose).homogeneous
    obj_x_axis_world = tf_world_obj[0:3, 0]
 
    new_obj_z_axis = tf_world_tote[0:3, 2]
    new_obj_y_axis = tfm.unit_vector(np.cross(new_obj_z_axis, obj_x_axis_world))
    new_obj_x_axis = tfm.unit_vector(np.cross(new_obj_y_axis, new_obj_z_axis))
 
    new_rot = np.array([new_obj_x_axis, new_obj_y_axis, new_obj_z_axis]).T
    pose_se3 = SE3(update_pose)
    pose_se3.rotation = new_rot
 
    quat_new = pose_se3.xyz_quat[3:7]
 
    new_obj_quat = tfm.quaternion_slerp(quat0=quat_old, quat1=quat_new, fraction=fraction)
 
    update_pose[3:] = new_obj_quat
 
    return update_pose

def build_collision(name, origin, dimensions, geometric_type, alpha):
    collision = PrimitiveGroup(name=name, origin=Pose(origin[0], origin[1], origin[2], origin[3], origin[4], origin[5], origin[6]))
    collision.color.a = alpha
    collision.primitives.append(GeometricPrimitive(type=geometric_type, dimensions=dimensions, relative_pose=Pose(0, 0, 0, 0, 0, 0, 1)))
    return collision 

def execute(self, inputs, outputs, gvm):
    self.logger.info("Hello {}".format(self.name))
    pl = get_planning_environment()
    planning_env = PlanningEnvironmentRos.from_ros_msg(pl)
    space_id = self.smart_data["space_id"]
    space_env = planning_env.get_workspace_ros(space_id)
    pallet_pose = inputs["pallet_pose"]
    
    #初始笼车坐标
    if space_id == "2":
        check_pose = self.smart_data["check_pose_2"]
        check_objcet_name = "collision_pallet_2"
        update_collision_name = "update_collision_1"
    elif space_id == "3":
        check_pose = self.smart_data["check_pose_3"]  
        check_objcet_name = "collision_pallet_3"   
        update_collision_name = "update_collision_2"
    
    #判断实际笼车位置和初始笼车位置       
    aixs_list = ["x","y","z"]
    for i in range(3):
        self.logger.info(f"{aixs_list[i]}方向偏差为{pallet_pose[i]-check_pose[i]}")  
    pallet_pose[2]+=0.01    
    pallet_pose[0]-=0.018
    if abs(pallet_pose[0]-check_pose[0])>0.05:
        raise "x方向偏差过大"  
    else:
        if space_id=="2" and pallet_pose[0]-check_pose[0]>0.03:
            raise "托盘2的+x方向偏差过大"
    if abs(pallet_pose[1]-check_pose[1])>0.05:
        raise "y方向偏差过大" 
    if abs(pallet_pose[2]-check_pose[2])>0.05:
        raise "z方向偏差过大"  
    
    vision_rotation = pallet_pose[3:7]
    z_angle = tfm.euler_from_quaternion(vision_rotation)[2]
    self.logger.info(f"z angle is {z_angle}")
    if z_angle>1.3 and z_angle<1.7:
        undate_rotation = vision_rotation
    else:
        undate_rotation = (SE3(vision_rotation)*SE3([0,0,1,0])).xyz_quat[3:7]


    #更新笼车工作空间
    update_pose = pallet_pose[0:3] +  undate_rotation   
    space_env_ros = space_env.to_ros_msg()
    #z轴垂直向上
    fraction = 1.0
    #work_space_ros = list(filter(lambda x:x.workspace_id==space_id,pl.workspaces))[0]
    tf_world_tote = SE3(pose_to_list(space_env_ros.pallet.top_origin)).homogeneous
    update_pose = modify_z_axis(update_pose, fraction, tf_world_tote)
    space_env_ros.bottom_pose = Pose(*update_pose)
    modify_workspace_of_environment(space_env_ros)  
    self.logger.info("更新码垛托盘到环境")

    # #通过笼车工作空间更新障碍物
    # check_flag = False
    # for collision_objcet in pl.collision_objects:
    #     if collision_objcet.name == update_collision_name:
    #         self.logger.info(f"更新笼车障碍物{update_collision_name}")
    #         collision_objcet.origin.x = pallet_pose[0]-0.815/2-0.12/2-0.03-0.015 
    #         collision_objcet.origin.y = pallet_pose[1]
    #         collision_objcet.origin.z = pallet_pose[2]+0.1  
    #         modify_primitive_group_of_environment(collision_objcet)   
    #         check_flag = True
    # if not check_flag:        
    #     #笼车干涉物
    #     self.logger.info(f"添加笼车障碍物")
    #     update_collision_pose = [pallet_pose[0]-0.815/2-0.12/2-0.015,pallet_pose[1],pallet_pose[2]+0.07]+undate_rotation
    #     update_collision = build_collision(name=update_collision_name, origin=update_collision_pose, dimensions=[1.2, 0.11,0.3], geometric_type=GeometricPrimitive.BOX, alpha=0.6)            
    #     add_primitive_group_of_environment([update_collision])    
    
    
    # collision_pose = inputs["collision_pose"]
    # self.logger.info(f"笼车围栏实际位置为{collision_pose}")
    # self.logger.info(f"通过托盘判断的位置为{pallet_pose[0],pallet_pose[1]+1.22/2+0.05/2,pallet_pose[2]+1.7/2}")
    # if abs(collision_pose[0]-pallet_pose[0])>0.05\
    # or abs(collision_pose[1]-(pallet_pose[1]+1.22/2+0.05/2))>0.05:
    #     raise "笼车围栏偏差过大"
    
    # #通过笼车工作空间更新笼车围栏障碍物
    # for collision_objcet in pl.collision_objects:
    #     if collision_objcet.name == check_objcet_name:
    #         self.logger.info(f"更新笼车围栏障碍物{check_objcet_name}")
    #         collision_objcet.origin = Pose(*collision_pose)  
    #         collision_objcet.origin.y+=0.025
    #         collision_objcet.origin.z=-1.2+1.7/2
    #         modify_primitive_group_of_environment(collision_objcet)   
    return "success"
