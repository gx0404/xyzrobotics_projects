
from xyz_env_manager.msg import Pose
from xyz_env_manager.client import modify_workspace_of_environment,get_planning_environment,modify_primitive_group_of_environment
from xyz_motion import PlanningEnvironmentRos
import tf.transformations as tfm
from xyz_motion import SE3
from xyz_env_manager.msg import PrimitiveGroup,Pose
from xyz_env_manager.msg import GeometricPrimitive
from xyz_env_manager.client import add_primitive_group_of_environment
import numpy as np
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
    pallet_vision_pose = inputs["pallet_vision_pose"]
    
    #初始托盘坐标
    pallet_init_pose = self.smart_data["pallet_pose_"+space_id]
    
    #判断实际笼车位置和初始笼车位置       
    aixs_list = ["x","y","z"]
    for i in range(3):
        self.logger.info(f"{aixs_list[i]}方向偏差为{pallet_vision_pose[i]-pallet_init_pose[i]}")   
        
    if abs(pallet_vision_pose[0]-pallet_init_pose[0])>0.08:
        raise "x方向偏差过大"    
    if abs(pallet_vision_pose[1]-pallet_init_pose[1])>0.08:
        raise "y方向偏差过大" 
    if abs(pallet_vision_pose[2]-pallet_init_pose[2])>0.05:
        raise "z方向偏差过大"  
    
    if abs(pallet_vision_pose[0]-pallet_init_pose[0])<0.02:
        self.logger.info(f"x方向偏差小于2cm可以直接使用视觉")
        update_pose_x = pallet_vision_pose[0] 
    elif pallet_vision_pose[0]-pallet_init_pose[0]>0.02 and pallet_vision_pose[0]-pallet_init_pose[0]<0.08:
        self.logger.info(f"x方向偏差大于+2cm,只偏置2cm")       
        update_pose_x = pallet_init_pose[0]+0.02
    elif pallet_vision_pose[0]-pallet_init_pose[0]<-0.02 and pallet_vision_pose[0]-pallet_init_pose[0]>-0.08:
        self.logger.info(f"x方向偏差大于-2cm,只偏置2cm") 
        update_pose_x = pallet_init_pose[0]-0.02

    if abs(pallet_vision_pose[1]-pallet_init_pose[1])<0.02:
        self.logger.info(f"y方向偏差小于2cm可以直接使用视觉")
        update_pose_y = pallet_vision_pose[1] 
    elif pallet_vision_pose[1]-pallet_init_pose[1]>0.02 and pallet_vision_pose[1]-pallet_init_pose[1]<0.08:
        self.logger.info(f"y方向偏差大于+2cm,只偏置2cm")       
        update_pose_y = pallet_init_pose[1]+0.02
    elif pallet_vision_pose[1]-pallet_init_pose[1]<-0.02 and pallet_vision_pose[1]-pallet_init_pose[1]>-0.08:
        self.logger.info(f"y方向偏差大于-2cm,只偏置2cm") 
        update_pose_y = pallet_init_pose[1]-0.02   
        
    
    vision_rotation = pallet_vision_pose[3:7]
    z_angle = np.rad2deg(tfm.euler_from_quaternion(vision_rotation)[2])
    self.logger.info(f"z angle is {z_angle}")
    
    if space_id in ["0","5"]:
        if z_angle>-8 and z_angle<8:
            undate_rotation = vision_rotation
        elif (z_angle>172 and z_angle<188) or (z_angle>-188 and z_angle<-172):
            undate_rotation = (SE3(vision_rotation)*SE3([0,0,1,0])).xyz_quat[3:7]
        else:
            raise "z方向旋转偏差过大"    
    elif space_id in ["1","2","3","4"]:
        if z_angle>82 and z_angle<98:
            undate_rotation = vision_rotation
        elif (z_angle>262 and z_angle<278) or (z_angle>-98 and z_angle<-82) :
            undate_rotation = (SE3(vision_rotation)*SE3([0,0,1,0])).xyz_quat[3:7]
        else:
            raise "z方向旋转偏差过大"          
        
    #更新笼车工作空间
    update_pose = [update_pose_x,update_pose_y] + [pallet_vision_pose[2]] +  undate_rotation   
    self.logger.info(f"更新的托盘的位姿为{update_pose}")
    space_env_ros = space_env.to_ros_msg()
    space_env_ros.bottom_pose = Pose(*update_pose)
    modify_workspace_of_environment(space_env_ros)  
    self.logger.info("更新码垛托盘到环境")
    
    return "success"
