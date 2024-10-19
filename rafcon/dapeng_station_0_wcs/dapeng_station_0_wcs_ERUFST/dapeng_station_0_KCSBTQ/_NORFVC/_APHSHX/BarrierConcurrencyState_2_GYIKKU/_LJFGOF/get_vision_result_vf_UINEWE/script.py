import copy
from datetime import datetime
import ipdb
import numpy as np
import json
from glob import glob
import os
import time
# from xyz_vision_flow.xyz_vision_bridge import XYZVisionBridge

from xyz_vision_lib.xyz_vision_bridge import XYZVisionBridge
from google.protobuf.json_format import MessageToDict


from rafcon.xyz_exception_base import XYZVisionException, XYZExceptionBase
from xyz_motion import PlanningEnvironmentRos
from xyz_env_manager.client import get_container_items, get_planning_environment, add_bottom_padding_workspace
from xyz_env_manager.client import add_container_items, clear_container_items, clear_container_all_items
from xyz_env_manager.msg import PointCloud, Pose
from xyz_motion import SE3,pose_to_list
import tf.transformations as tfm
from xyz_motion import WorkspaceRos
from xyz_motion import FormattedPlanBox, FormattedRealBox, SE3

from xyz_motion import RobotDriver
from xyz_io_client.io_client import set_digit_output

from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
from rospy import Time


"""obb 碰撞检测"""
class OBB:
    def __init__(self, center, half_size, angle):
        self.center = np.array(center)
        self.half_size = np.array(half_size)
        self.angle = angle
        self.axes = self.compute_axes()

    def compute_axes(self):
        cos_a = np.cos(self.angle)
        sin_a = np.sin(self.angle)
        u1 = np.array([cos_a, sin_a])      # 主轴（长边方向）
        u2 = np.array([-sin_a, cos_a])     # 副轴（短边方向）
        return u1, u2

    def get_corners(self):
        u1, u2 = self.axes
        corner1 = self.center + self.half_size[0] * u1 + self.half_size[1] * u2
        corner2 = self.center + self.half_size[0] * u1 - self.half_size[1] * u2
        corner3 = self.center - self.half_size[0] * u1 - self.half_size[1] * u2
        corner4 = self.center - self.half_size[0] * u1 + self.half_size[1] * u2
        return [corner1, corner2, corner3, corner4]

def project_obb(obb, axis):
    corners = obb.get_corners()
    projections = [np.dot(corner, axis) for corner in corners]
    return min(projections), max(projections)

def obb_overlap(obb1, obb2):
    axes = [obb1.axes[0], obb1.axes[1], obb2.axes[0], obb2.axes[1]]
    
    for axis in axes:
        min1, max1 = project_obb(obb1, axis)
        min2, max2 = project_obb(obb2, axis)
        
        if max1 < min2 or max2 < min1:
            return False
    
    return True

#通过obb计算是否碰撞
def is_collision(check_item,other_items):
    check_item_pose = pose_to_list(check_item.origin)
    l_1,w_1,h_1 = check_item.primitives[0].dimensions
    check_item_angle = tfm.euler_from_quaternion(check_item_pose[3:7])[2]
    #缩小物料尺寸,避免碰撞检测误差
    check_item_obb = OBB(center=(check_item_pose[0], check_item_pose[1]), half_size=(l_1/2-0.007,w_1/2-0.007), angle=check_item_angle)
    check_collision = False
    for item in other_items:
        check_collision = True
        item_pose = pose_to_list(item.origin)
        l_2,w_2,h_2 = item.primitives[0].dimensions
        item_angle = tfm.euler_from_quaternion(item_pose[3:7])[2]
        #缩小物料尺寸,避免碰撞检测误差
        item_obb = OBB(center=(item_pose[0], item_pose[1]), half_size=(l_2/2-0.007,w_2/2-0.007), angle=item_angle)
        
        if not obb_overlap(check_item_obb,item_obb):
            check_collision = False
        if check_collision:
            break
    return check_collision   

#过滤得到底层箱子
def filter_bottom_items(items,row_flag=True):
    #两种模式,一种只取最低层，另一种则是每列箱子的最低层
    if row_flag:
        combined_data = {}
        for item in items:
            #建立x,y坐标的键，同一列箱子xy坐标一致
            key = (round(item.origin.x,2), round(item.origin.y,2))
            if key not in combined_data.keys():
                #判断原先字典是否有xy近似的key的标志flag
                check_key_flag = False
                for check_key in combined_data.keys():
                    #判断绝对值是否小于0.015，如果xy都小于0.015，则认为是同列箱子
                    if abs(item.origin.x-check_key[0])<0.015 and abs(item.origin.y-check_key[1])<0.015:    
                        check_key_flag = True
                        break
                #如果不存在标志,则说明是个新列         
                if not check_key_flag:                    
                    combined_data[key] = item
                #如果存在,则说明是老列,则需要判断是否保留z最小的实例   
                else:
                    if item.origin.z < combined_data[check_key].origin.z:
                        combined_data[check_key] = item                  
            else:   
                # 只保留Z最小的类实例
                if item.origin.z < combined_data[key].origin.z:
                    combined_data[key] = item

        new_items = list(combined_data.values())
    #只考虑最低列,不考虑每列层数不同      
    else:    
        min_z = min(i.origin.z for i in items)
        new_items = list(filter(lambda x:abs(x.origin.z-min_z)<0.1,items))    
    return new_items

# modify group x axis
# barcode direction: 4: -x; 2: -y
def modify_XY_axis(vision_results, workspace, barcode_direction):
    
    tf_world_tote = workspace.get_bottom_pose().homogeneous
    if barcode_direction not in [2, 4]:
        return vision_results
    def rotate_z_by_180(vision_result):
        quat_list = vision_result["pose"][3:]
        Rz = tfm.rotation_matrix(np.pi, (0, 0, 1))
        Rq = tfm.quaternion_matrix(quat_list)
        new_quat_list = tfm.quaternion_from_matrix(Rq.dot(Rz)).tolist()
        assert len(new_quat_list) == 4
        vision_result["pose"][3:] = new_quat_list
        return vision_result

    for index in range(len(vision_results)):
        tf_world_obj = SE3(vision_results[index]["pose"]).homogeneous
        tf_tote_obj = np.linalg.inv(tf_world_tote).dot(tf_world_obj)
        trans_tote_obj = tf_tote_obj[0:3, 3]
        tf_tote_objX = tf_tote_obj[0:3, 0]
        tf_tote_objY = tf_tote_obj[0:3, 1]
        if barcode_direction == 4:
            if trans_tote_obj.dot(tf_tote_objX) > 0:
                vision_results[index] = rotate_z_by_180(vision_results[index])
        elif barcode_direction == 2:
            if trans_tote_obj.dot(tf_tote_objY) > 0:
                vision_results[index] = rotate_z_by_180(vision_results[index])
    return vision_results

# modify vision result z axis
def modify_z_axis(vision_result, fraction, tf_world_tote):
    """
    this is a funciton to modify the vision result z axis

    Args:
        vision_result: original vision result
        fraction: the fraction between [0, 1], 0 means dont modify, and 1 means vision z axis parallel to workspace z axis
        worskspace_id: vision workspace id
    Returns:
        new vision result
    Exception:

    """
    for index in range(len(vision_result)):
        quat_old = vision_result[index]["pose"][3:]
        tf_world_obj = SE3(vision_result[index]["pose"]).homogeneous
        obj_x_axis_world = tf_world_obj[0:3, 0]

        new_obj_z_axis = tf_world_tote[0:3, 2]
        new_obj_y_axis = tfm.unit_vector(np.cross(new_obj_z_axis, obj_x_axis_world))
        new_obj_x_axis = tfm.unit_vector(np.cross(new_obj_y_axis, new_obj_z_axis))

        new_rot = np.array([new_obj_x_axis, new_obj_y_axis, new_obj_z_axis]).T
        
        pose_se3 = SE3(vision_result[index]["pose"])
        pose_se3.rotation = new_rot

        quat_new = pose_se3.xyz_quat[3:7]

        new_obj_quat = tfm.quaternion_slerp(quat0=quat_old, quat1=quat_new, fraction=fraction)

        vision_result[index]["pose"][3:] = new_obj_quat


    return vision_result


def execute(self, inputs, outputs, gvm):
    """ 
    This state calls xyz vision service to get object information. 

    Args:
        Inputs Data:
            sku_info (dict): Default value (None).
                The dimension check is disabled if sku_info is None.
            
            capture_res (dict):
                if flag_caputre_image is false, this value will come from other modules.

        Outputs Data:
            collision_cloud: (xyz_env_manager.msg._PointCloud.PointCloud): Default value (None).
                Collision cloud after capturing image, which is used for mix_depalletize.

        Gvm: 
            vision_bridge (xyz_vision_flow.xyz_vision_bridge.XYZVisionBridge) {Get}: Must be created in init_all.
                vision_bridge created in init_all state. Should be removed in the future.

            

    Properties Data:
        shrunk_size, (float): Default value (0.0).
            shrunk size of each box
        
        workspace_id, (unicode): Default value ("0").
            workspace id to call vision service

        vision_service, (unicode): Default value ("calculate_object_poses").
            vision service name

    Raises:
        XYZExceptionBase: Remote helper not found.
        XYZVisionException: Error getting latest image paths.
        XYZExceptionBase: Remote helper failed.
        XYZVisionException: Vision check failed after the labeling.
        XYZVisionException: Error from vision bridge.
        XYZVisionException: Vision dimension cannot pass dimension check.
        XYZExceptionBase: Shrunk_size should be between 0m and 0.05m.
        XYZExceptionBase: Appointed workspace does not exist in environment.

    Outcomes:
        1: empty
        0: success
        -1: aborted
        -2: preempted
    """
    self.logger.info("Running {}({})({})".format(self.name, self.unique_id, self.smart_data["comment"]))
    DEFAULT_OFFLINE_VISION_PATH = "/home/xyz/xyz_app/app/vision_offline/"
    EPSILON = 1e-4
    ## TODO remove vision bridge
    if not gvm.get_variable("vision_bridge", per_reference=True, default=None):

        vision_bridge = XYZVisionBridge()
        gvm.set_variable("vision_bridge", vision_bridge, per_reference = True)
    else:
        vision_bridge = gvm.get_variable("vision_bridge", per_reference = True)
    

    if not gvm.get_variable("motion_payload", per_reference=True, default=None):
        planning_env_msg = get_planning_environment()
        planning_env = PlanningEnvironmentRos.from_ros_msg(planning_env_msg)
    else:
        last_payload = gvm.get_variable("motion_payload", per_reference=True, default=None)
        planning_env = PlanningEnvironmentRos.from_ros_msg(last_payload["planning_environment"])
        
    workspace_id = self.smart_data["workspace_id"]
    
    vision_id = workspace_id    
    
    mode = self.smart_data["mode"]
    data_source_path = self.smart_data["data_folder"]["data_source_path"]
    flag_capture_image = self.smart_data["flag_capture_image"]

    #this gvm must be deleted before building primitives
    if gvm.variable_exist("height_previous_" + workspace_id):
        gvm.delete_variable("height_previous_" + workspace_id)

    if mode not in ["RELEASE", "DEBUG"]:
        raise Exception("please input correct mode(请输入正确的模式名字 RELEASE 或者 DEBUG)")
    
    
    sku_info = inputs["sku_info"]
    camera_num = inputs["camera_num"]
    if camera_num == 1:
        capture_service_list = ["one_camera_capture"]
    elif camera_num == 2:
        capture_service_list = ["two_camera_capture_0","two_camera_capture_1"]
    elif camera_num == 3:
        capture_service_list = ["three_camera_capture_0","three_camera_capture_1","three_camera_capture_2"]
    else:
        raise Exception("错误的相机数量")
    
    for index,capture_service in enumerate(capture_service_list):
        start_time = time.time()
        self.logger.info(f"拍照service: {capture_service}开始")
        while not self.preempted:
            if index>=2:                 
                ftr = vision_bridge.async_run(int(vision_id),capture_service)    
                #capture_res = vision_bridge.run(int(vision_id),capture_service)   

            else:
                ftr = vision_bridge.async_run(int(vision_id),capture_service) 
                #capture_res = vision_bridge.run(int(vision_id),capture_service)   
                                                       
            #获取异步结果
            current_time = time.time()
            while not self.preempted:
                if time.time()-current_time>10:
                    self.logger.info(f"异步获取视觉结果超时")
                    raise XYZExceptionBase("20010", "OpenCameraFailed")
                try:
                    capture_res = ftr.get()
                    if index>=2: 
                        if camera_num == 2:
                            merge_ftr = vision_bridge.async_run(int(vision_id), "two_camera_merge_"+str(index-2)) 
                            self.logger.info(f"调用合并service: {'two_camera_merge_'+str(index-2)}")          
                            merge_ftr.get()                
                        else:    
                            merge_ftr = vision_bridge.async_run(int(vision_id), "three_camera_merge_"+str(index-2)) 
                            self.logger.info(f"调用合并service: {'three_camera_merge_'+str(index-2)}")    
                            merge_ftr.get()    
                    break
                except Exception as e:
                    self.logger.info(f"拍照service: {capture_service},异步拍照结果还未出现,重新获取")
                    self.logger.info(f"{e}")
                    self.preemptive_wait(0.1)
                    continue
            
            #capture_res = ftr
            if (time.time()-start_time) > self.smart_data["time_out"]:
                self.logger.info(f"拍照service: {capture_service}超时")
                raise XYZExceptionBase("20010", "OpenCameraFailed")
            try:
                capture_res = MessageToDict(capture_res)
                if capture_res.get("error") == 0:
                    break
                else:
                    self.logger.warning("Try to re-connect camera, Please wait!")   
                    self.logger.info(f"拍照service: {capture_service},拍照失败,重新拍照")
                    self.logger.warning(capture_res)  
                    self.preemptive_wait(2) 
            except:
                self.preemptive_wait(2)
              
    if camera_num==2:
        merge_ftr = vision_bridge.async_run(int(vision_id), "two_camera_merge_0")    
        self.logger.info(f"调用合并service: two_camera_merge_0")     
        merge_ftr.get()    
    elif camera_num==3:                
        merge_ftr = vision_bridge.async_run(int(vision_id), "three_camera_merge_1")    
        self.logger.info(f"调用合并service: three_camera_merge_1")  
        merge_ftr.get()
    vision_service_dict = {
        1:"one_camera_calculate_object_poses",
        2:"two_camera_calculate_object_poses",
        3:"three_camera_calculate_object_poses"
    }    
    
    ftr = vision_bridge.async_run(int(vision_id),vision_service_dict[camera_num])    
    #vision_result_raw = vision_bridge.run(int(vision_id),vision_service_dict[camera_num]) 
    #获取异步结果
    current_time = time.time()
    while not self.preempted:
        if time.time()-current_time>30:
            self.logger.info(f"异步获取识别结果超时")
            raise "异步获取识别结果超时"
        try:
            vision_result_raw = ftr.get()
            break
        except Exception as e:
            self.logger.info(f"异步识别结果还未出现,重新获取")
            self.logger.info(f"{e}")
            self.preemptive_wait(0.1)
            continue     

    def construct_vision_result(vision_result_raw, ts):
        def convert_cloud(cloud_proto):
            header = Header(
                seq=cloud_proto.header.seq,
                stamp=Time(),
                frame_id=cloud_proto.header.frame_id,
            )
            fields = []
            for f in cloud_proto.fields:
                fields.append(
                    PointField(name=f.name, offset=f.offset, datatype=f.datatype, count=f.count)
                )
            return PointCloud2(
                header=header,
                height=cloud_proto.height,
                width=cloud_proto.width,
                is_dense=cloud_proto.is_dense,
                is_bigendian=cloud_proto.is_bigendian,
                fields=fields,
                point_step=cloud_proto.point_step,
                row_step=cloud_proto.row_step,
                data=cloud_proto.data,
            )

        vision_result = {
            "error": vision_result_raw.error,
            "results": [],
            "cloud": None,
            "timestamp": ts
        }
        for i, item in enumerate(vision_result_raw.primitives_3d):
            pose = [
                item.pose.position.x,
                item.pose.position.y,
                item.pose.position.z,
                item.pose.orientation.x,
                item.pose.orientation.y,
                item.pose.orientation.z,
                item.pose.orientation.w,
            ]
            dimension = [
                sku_info["length"],
                sku_info["width"],
                sku_info["height"],
            ]
            info = {
                "id": i,
                "pose": pose,
                "dimension": dimension,
                "name": "box",
                "score": item.score,
                "grasp_poses": [],
                "timestamp": 0,
                "vision_id": i,
            }
            vision_result["results"].append(info)
        
        if vision_result_raw.cloud:
            vision_result["cloud"] = convert_cloud(vision_result_raw.cloud)
        return vision_result

    ts = capture_res.get("timestamp")
    
    vision_result = construct_vision_result(vision_result_raw, ts)        
    if vision_result.get("error"):
        self.logger.info(f"获取视觉结果失败")
        raise XYZExceptionBase("10001", "获取视觉结果失败")   
    # camera_ids = vision_bridge.get_camera_ids(vision_id).get('results')
    camera_ids = ["-1"]
    results = copy.copy(vision_result.get("results"))
        

    # add point cloud
    if vision_result.get("cloud"):
        self.logger.info("视觉获取到未识别成sku的点云(vision get cloud that not belong to primitive)")
        vision_cloud = vision_result.get("cloud")
        collision_cloud = PointCloud()
        collision_cloud.name = "collision_cloud"
        collision_cloud.point_cloud = vision_cloud
        collision_cloud.origin  = Pose(0, 0, 0, 0, 0, 0, 1)
        collision_cloud.grid_size = 0.005
        collision_cloud.ignore_collision = False
        outputs["collision_cloud"] = collision_cloud

    results = copy.copy(vision_result.get("results"))
    ## add modify vision reuslt function
    modify_z_axis_properties = self.smart_data["modify_Z_axis"]
    if modify_z_axis_properties["enable"]:
        fraction = modify_z_axis_properties["fraction"]
        workspace = planning_env.get_workspace_ros(workspace_id)
        tf_world_tote = workspace.get_bottom_pose().homogeneous
        vision_result["results"]  = modify_z_axis(results, fraction, tf_world_tote)
    
    ## modify xy axis
    workspace = planning_env.get_workspace_ros(workspace_id)
    global tf_base_space
    tf_base_space = workspace.get_bottom_pose()        
    if inputs["sku_info"]:
        if "barcode_direction" not in inputs["sku_info"].keys():
            self.logger.warning("Barcode direction is not set, using default value 0")
            barcode_direction = 0
        else:
            barcode_direction = inputs["sku_info"]["barcode_direction"]
        
        vision_result["results"] = modify_XY_axis(results, workspace, barcode_direction)
    else:
        self.logger.warning("No input sku info, Notice that barcode direction is not set!")

        
    list(map(lambda info: info.update({"timestamp": vision_result.get("timestamp")}), results))
    list(map(lambda info: info.update({"sku_info": inputs["sku_info"]}), results))
    
    shrunk_size = self.smart_data["shrunk_size"]
    if not (shrunk_size >= 0 and shrunk_size <= 0.05):
            raise XYZExceptionBase(error_code="E9998",
                        error_msg="shrunk_size[{}] should be between 0m and 0.05m".format(shrunk_size))
                 
    if len(results) == 0:
        self.logger.info(f"视觉识别为空,但是存在抓取搜索的路径")
        raise XYZExceptionBase("10001", "获取视觉结果失败")          

    
    minimal_sku_height = self.smart_data["mix_depalletize_minimal_sku_height"]
    if inputs["sku_info"]:
        
        l, w, h = inputs["sku_info"]["length"], inputs["sku_info"]["width"], inputs["sku_info"]["height"]
        #h = min(vision_result["results"][0]["pose"][2] - workspace.get_bottom_pose().xyz_quat[2], inputs["sku_info"]["height"])
        
        ## default we assume this is single depalletize
        if minimal_sku_height == 0.0:
            minimal_sku_height = h

        def check_dim(info):
            vision_dim = np.sort(np.array(info["dimension"][:2]))[::-1]
            sku_dim = np.array([l,w])
            return np.allclose(vision_dim,sku_dim, rtol=0, atol=0.05)

        if self.smart_data["check_dimension"]:
            if (not all(map(check_dim, vision_result.get("results", [])))):
                vision_result["error"] = 1
                vision_result["error_msg"] = "RAFCON Vision error: {}".format(
                        "Vision dimension cannot pass dimension check.")
                error_msg = "Vision dimension cannot pass dimension check."
                raise XYZVisionException(error_code="E0609",
                                    error_msg=error_msg,
                                    camera_ids=camera_ids) 

            list(map(lambda info: info.update({"dimension": [l-shrunk_size+EPSILON, w-shrunk_size+EPSILON, h+EPSILON]}), results)) #shrunk size to get collision checking work
        else:
            def only_update_height(info):
                info["dimension"][0] = info["dimension"][0] - shrunk_size + EPSILON
                info["dimension"][1] = info["dimension"][1] - shrunk_size + EPSILON
                info["dimension"][2] = h + EPSILON

            list(map(only_update_height, results)) #shrunk size to get collision checking work

    
    ## update sku_info using top box
    results.sort(key=lambda info: info["pose"][2], reverse=True)
    sku_info = results[0]["sku_info"]
    # No input sku_info
    if not sku_info:
        self.logger.warning("Attention! NO input sku information.")
        sku_info = {}
    sku_info["length"] = results[0]["dimension"][0]
    sku_info["width"] = results[0]["dimension"][1]
    sku_info["height"] = results[0]["dimension"][2]
    outputs["sku_info"] = sku_info

    list(map(lambda info: info.update({"vision_id": info["id"]}), results))
    self.logger.debug(results)
    self.logger.info(f"视觉识别到{len(results)}个料箱")
    
    init_results = copy.deepcopy(results)
    box_id = int(time.time()/10000000)    
    overlapping_heihgt = inputs["sku_info"]["overlapping_heihgt"]    
    
    for result in init_results:
        tf_space_box_real = (tf_base_space.inv())*SE3(result["pose"])
        layer = round(tf_space_box_real.xyz_quat[2]/(sku_info["height"]-overlapping_heihgt))
        self.logger.info(f"视觉pose为{result['pose']},当前层数为{layer}")
        for i in range(1,layer): 
            append_result = copy.deepcopy(result)    
            box_id+=1          
            append_result["id"] = box_id
            append_result["pose"][2]-=i*(sku_info["height"]-overlapping_heihgt)
            append_result["timestamp"] = int(time.time())
            results.append(append_result)   
             
    items = planning_env.get_container_items(vision_id)
    if not items:
        raise Exception("环境已无箱子")
    
    #在单独日志文件记录识别结果，避免xtf日志过长
    f = open('/home/xyz/xyz_app/app/rafcon/eye_to_hand_vison_check.log', 'w')
    #更新环境中的箱子,并做匹配
    update_items = []      
    for item in items:
        tf_base_box_old = SE3(pose_to_list(item.origin))
        for result in results:
            tf_base_box_new = SE3(result["pose"])
            check_list = list(filter(lambda x:abs(tf_base_box_old.xyz_quat[x]-tf_base_box_new.xyz_quat[x])<0.05,[0,1]))
            check_list_z = list(filter(lambda x:abs(tf_base_box_old.xyz_quat[x]-tf_base_box_new.xyz_quat[x])<0.05,[2])) 
            if len(check_list)==2 and check_list_z:
                item.origin = Pose(*result["pose"])
                update_items.append(item)
                f.write(f"视觉pose为{result['pose']}\n")
                f.write(f"环境pose为{tf_base_box_old.xyz_quat}\n")
                break
            
    update_box_ids = [i.additional_info.values[-3] for i in update_items]    
    self.logger.info(f"顶置相机更新的箱子为{update_box_ids}") 
    f.close()
    
    #视觉识别结果和环境中箱子数量不一致，说明识别偏差过大
    if len(results)!=len(update_items):
        self.logger.info(f"匹配到环境的箱子和视觉箱子数量不一致,可能二次识别偏差值过大")  
        
    clear_container_items(vision_id,[i.name for i in update_items])
    add_container_items(vision_id,update_items)    
    
    #通过输入拿到下次抓取的箱子id
    next_pick_item_id = inputs["next_pick_item_id"]
       
    if next_pick_item_id in [i.additional_info.values[-3] for i in update_items]:
        self.logger.info(f"顶置相机视觉结果已识别到目标抓取箱{next_pick_item_id}")   
        planning_env_msg = get_planning_environment()
        planning_env = PlanningEnvironmentRos.from_ros_msg(planning_env_msg)
        container_items = planning_env.get_container_items(vision_id)
        bottom_items = filter_bottom_items(container_items)
        for bottom_item in bottom_items:
            check_items = copy.deepcopy(bottom_items)
            check_items.remove(bottom_item)
            bottom_item_id = bottom_item.additional_info.values[-3]
            check_items_id = [i.additional_info.values[-3] for i in check_items]        
            self.logger.info(f"检测箱子{bottom_item_id} 与 {check_items_id} 是否存在碰撞")
            if is_collision(bottom_item, check_items):
                self.logger.info(f"视觉检测到碰撞")
                raise XYZExceptionBase("10022", "视觉检测到碰撞")            
        return "success"
    else:
        self.logger.info(f"顶置相机视觉结果未识别到目标抓取箱{next_pick_item_id}")  
        raise XYZExceptionBase("10001", "获取视觉结果失败") 
