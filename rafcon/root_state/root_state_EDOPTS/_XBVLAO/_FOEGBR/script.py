
from xyz_env_manager.msg import Pose
from xyz_env_manager.client import modify_workspace_of_environment,get_planning_environment,modify_primitive_group_of_environment
from xyz_motion import PlanningEnvironmentRos
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
    elif space_id == "3":
        check_pose = self.smart_data["check_pose_3"]  
        check_objcet_name = "collision_pallet_3"   
    
    #判断实际笼车位置和初始笼车位置       
    aixs_list = ["x","y","z"]
    for i in range(3):
        self.logger.info(f"{aixs_list[i]}方向偏差为{check_pose[i]-pallet_pose[i]}")  
    pallet_pose[2]+=0.017    
    if abs(check_pose[0]-pallet_pose[0])>0.05:
        raise "x方向偏差过大"     
    if abs(check_pose[1]-pallet_pose[1])>0.05:
        raise "y方向偏差过大" 
    if abs(check_pose[2]-pallet_pose[2])>0.05:
        raise "z方向偏差过大"  
    
    #更新笼车工作空间
    update_pose = pallet_pose[0:3] +  [0.0, 0.0, 0.707, 0.707]    
    space_env_ros = space_env.to_ros_msg()
    space_env_ros.bottom_pose = Pose(*update_pose)
    modify_workspace_of_environment(space_env_ros)  
    self.logger.info("更新码垛托盘到环境")    
    
    #通过笼车工作空间更新笼车围栏障碍物
    for collision_objcet in pl.collision_objects:
        if collision_objcet.name == check_objcet_name:
            self.logger.info(f"更新笼车围栏障碍物{check_objcet_name}")
            collision_objcet.origin.x = pallet_pose[0]  
            collision_objcet.origin.y = pallet_pose[1]+1.22/2+0.11/2 
            collision_objcet.origin.z = pallet_pose[2]+1.7/2    
            modify_primitive_group_of_environment(collision_objcet)   
    return "success"
