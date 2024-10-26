from xyz_motion import PlanningEnvironmentRos,pose_to_list,SE3
from xyz_env_manager.client import get_planning_environment
import tf.transformations as tfm
import numpy as np
from xyz_env_manager.msg import Pose
from xyz_env_manager.client import clear_container_all_items,add_container_items

def execute(self, inputs, outputs, gvm):
    self.logger.info("Hello {}".format(self.name))
    planning_env_msg = get_planning_environment()
    planning_env = PlanningEnvironmentRos.from_ros_msg(planning_env_msg)
    #获取缓存区所有箱子
    place_container_items = planning_env.get_container_items("1")
    #获取数据库缓存区信息
    cache_pallet_tote_data = inputs["cache_pallet_tote_data"]
    if len(place_container_items)!=len(cache_pallet_tote_data):
        self.logger.info(f"缓存区数据长度和视觉箱子数量不一致")
        raise f"缓存区数据长度和视觉箱子数量不一致"
    
    #获取缓存托盘坐标
    place_workspace_ros = planning_env.get_workspace_ros("1")
    tf_base_place_space = place_workspace_ros.get_bottom_pose()
    pose_dict = inputs["pose_dict"]
    #匹配当前箱子条码     
    for container_item in place_container_items:
        tf_base_box_real = SE3(pose_to_list(container_item.origin))
        tf_space_box_real = (tf_base_place_space.inv())*tf_base_box_real
        space_box_pose_real = tf_space_box_real.xyz_quat
        self.logger.info(f"缓存视觉料箱坐标XYZ坐标为 {space_box_pose_real[0:3]}")
        for key,values in pose_dict.items():    
            check_list = list(filter(lambda x:abs(space_box_pose_real[x]-values[x])<0.07,[0,1]))  
            check_list_z = list(filter(lambda x:abs(space_box_pose_real[x]-values[x])<0.05,[2]))           
            if len(check_list)==2 and check_list_z:
                box_id = key
                break
            else:
                box_id = -1       
        if box_id<0:
            self.logger.error("未能匹配上位置号,请检查视觉数据和缓存数据")
            raise "未能匹配上位置号,请检查视觉数据和缓存数据"
        else:    
            self.logger.info(f"匹配到位置id为{box_id}")    
        #添加box_id
        box_id = str(box_id) 
        #添加信息到环境
        if box_id in cache_pallet_tote_data.keys():
            #更新料箱ID到环境
            if "box_id" in container_item.additional_info.keys:
                index = container_item.additional_info.keys.index("box_id")
                container_item.additional_info.values[index] = box_id
            else:
                container_item.additional_info.keys.append("box_id")   
                container_item.additional_info.values.append(box_id)   
            #更新条码到环境       
            if "barcode" in container_item.additional_info.keys:
                index = container_item.additional_info.keys.index("barcode")
                container_item.additional_info.values[index] = cache_pallet_tote_data[box_id]["barcode"]       
            else:    
                container_item.additional_info.keys.append("barcode")
                container_item.additional_info.values.append(cache_pallet_tote_data[box_id]["barcode"]) 
            #更新to_ws到环境,主要是因为一致性,实际用处不大       
            if "barcode" in container_item.additional_info.keys:
                index = container_item.additional_info.keys.index("to_ws")
                container_item.additional_info.values[index] = ""    
            else:    
                container_item.additional_info.keys.append("to_ws")
                container_item.additional_info.values.append("")   
            #判断更新料箱条码朝向
            real_box_rotation_angle = tfm.euler_from_quaternion(tf_base_box_real.xyz_quat[3:7])
            cache_box_rotation_angle = tfm.euler_from_quaternion(cache_pallet_tote_data[box_id]["pose_rotation"])
            self.logger.info(f"{box_id}号视觉角度{real_box_rotation_angle},数据库角度{cache_box_rotation_angle}")
            if np.rad2deg(abs(real_box_rotation_angle[0]-cache_box_rotation_angle[0]))>5\
                or np.rad2deg(abs(real_box_rotation_angle[1]-cache_box_rotation_angle[1]))>5:
                self.logger.info(f"xy角度偏差大于5°,存在倾斜")
                raise f"xy角度偏差大于5°,存在倾斜"
            z_angle = np.rad2deg(abs(real_box_rotation_angle[2]-cache_box_rotation_angle[2]))
            if z_angle<10 or (360-z_angle)<10:
                self.logger.info(f"{box_id}号料箱不需要旋转,角度偏差小于10°")
            elif z_angle>170 and z_angle<190:   
                tf_base_box_real = tf_base_box_real*SE3([0,0,0,0,0,1,0])  
                container_item.origin = Pose(*tf_base_box_real.xyz_quat)
                self.logger.info(f"{box_id}号料箱在170,190范围内,需要根据数据库角度旋转朝向") 
            else:
                self.logger.info(f"视觉角度,数据库角度为{z_angle},z角度偏差大于10°") 
                raise f"z角度偏差大于10°"   
            
            #偏置
            row = gvm.get_variable("row", per_reference=False, default=None)
            if row==5:
                row_id = int(box_id)%row
                lay_id = int(box_id)//row                  
                # if row_id in [1,2,3]:                    
                #     tf_base_box_real = SE3([0.002,-0.002,0,0,0,0,1])*tf_base_box_real
                # elif row_id in [0,4]:
                #     tf_base_box_real = tf_base_box_real*SE3([0.00,0.00,0,0,0,0,1])     
            elif row==9:
                row_id = int(box_id)%row
                lay_id = int(box_id)//row
                if row_id in [1,2]:
                    tf_base_box_real = SE3([0.002,-0.001,0,0,0,0,1])*tf_base_box_real
                elif row_id in [3,4]:     
                    tf_base_box_real = SE3([0.002,-0.001,0,0,0,0,1])*tf_base_box_real
                elif row_id in [5]:   
                    tf_base_box_real = SE3([0.007,-0.002,0,0,0,0,1])*tf_base_box_real                     
                elif row_id in [6]:  
                    tf_base_box_real = SE3([0.007,-0.002,0,0,0,0,1])*tf_base_box_real                                                              
                elif row_id in [7]:
                    tf_base_box_real = SE3([0.005,-0.003,0,0,0,0,1])*tf_base_box_real                   
                elif row_id in [8]:
                    tf_base_box_real = SE3([0.006,0.00,0,0,0,0,1])*tf_base_box_real                      
                elif row_id in [0]:
                    tf_base_box_real = SE3([0.006,0.00,0,0,0,0,1])*tf_base_box_real                            
            else:
                raise "无效的row"     
            container_item.origin = Pose(*tf_base_box_real.xyz_quat)               
        else:
            self.logger.info(f"托盘数据缺少位置号{box_id}")
            raise f"托盘数据缺少位置号"    
    clear_container_all_items("1")
    add_container_items("1",place_container_items)

    return "success"
