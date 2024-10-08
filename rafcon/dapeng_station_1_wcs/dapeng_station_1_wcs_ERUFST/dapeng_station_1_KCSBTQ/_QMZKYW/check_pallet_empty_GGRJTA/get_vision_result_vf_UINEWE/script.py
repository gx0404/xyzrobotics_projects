import copy
from datetime import datetime
from signal import raise_signal
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
from xyz_motion import SE3
import tf.transformations as tfm
from xyz_motion import WorkspaceRos
from xyz_motion import FormattedPlanBox, FormattedRealBox, SE3

from xyz_motion import RobotDriver
from xyz_io_client.io_client import set_digit_output


from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
from rospy import Time


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

    vision_service = self.smart_data["vision_service"]

    if flag_capture_image:
        start_time = time.time()
        eye_in_hand = self.smart_data["eye_in_hand"]  
        while not self.preempted:
            if eye_in_hand:
                r = RobotDriver(0)
                scan_pose = r.get_cartpose().xyz_quat
                ftr = vision_bridge.async_run(int(vision_id), "capture_images_1",info=json.dumps({"tf_world_hand":scan_pose}))
            else:
                ftr = vision_bridge.async_run(int(vision_id), "capture_images_1")            
                      
            #获取异步结果
            current_time = time.time()
            while not self.preempted:
                if time.time()-current_time>10:
                    self.logger.info(f"异步获取视觉结果超时")
                    raise XYZExceptionBase("20010", "OpenCameraFailed")
                try:
                    capture_res = ftr.get()
                    break
                except Exception as e:
                    self.logger.info(f"{e}")
                    self.preemptive_wait(0.1)
                    continue
                
            if (time.time()-start_time) > self.smart_data["time_out"]:
                raise XYZExceptionBase("20010", "OpenCameraFailed")
            try:
                capture_res = MessageToDict(capture_res)
                if capture_res.get("error") == 0:
                    break
                else:
                    self.logger.warning("Try to re-connect camera, Please wait!")   
                    self.logger.warning(capture_res)  
                    self.preemptive_wait(2) 
            except:
                self.preemptive_wait(2)
                
    else:
        capture_res = inputs["capture_res"]
              
    ftr = vision_bridge.async_run(int(vision_id),vision_service)
    #vision_result_raw = vision_bridge.run(int(vision_id), vision_service)
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

    from xyz_msgs.vision_msgs import VisionSrv_pb2
    primitives_3d = VisionSrv_pb2.Request().primitives_3d
    for result in vision_result_raw.primitives_3d:
        request = VisionSrv_pb2.Primitive3D()  #将3Dprimitves信息转换成Protobuf格式用来和视觉服务通讯
        request.name = result.name
        request.id = result.id
        request.pose.position.x = result.pose.position.x
        request.pose.position.y = result.pose.position.y
        request.pose.position.z = result.pose.position.z 
        request.pose.orientation.x = result.pose.orientation.x
        request.pose.orientation.y = result.pose.orientation.y
        request.pose.orientation.z = result.pose.orientation.z
        request.pose.orientation.w = result.pose.orientation.w

        request.dimension.x = sku_info["length"]
        request.dimension.y = sku_info["width"]
        request.dimension.z = sku_info["height"]
        request.score = result.score
        primitives_3d.extend([request])



    ftr = vision_bridge.async_run(int(vision_id), "check_primitive_cloud",primitives_3d=primitives_3d)
    check_res = ftr.get()
    print(check_res)
    if not check_res.error == 0:        
        self.logger.error(f"完整性校验失败")
        raise "完整性校验失败"
        
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

    #完整性校验
        

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

    #使用get height去判断是否为空
    height_res = vision_bridge.run(int(vision_id), "get_safe_height")
    height = float(height_res.info)
    if str(vision_id) in ["2","3"]:
        height-=0.025    
    self.logger.info(f"height is {height}")
    if height<0.05:
        if len(results) == 0:
            self.logger.info(f"高度小于0.05，视觉判断为空")
            return "empty"
        else:
            self.logger.info("视觉高度小于0.05，但是视觉识别到料箱")
            raise "视觉高度小于0.05，但是视觉识别到料箱"    


    #这个结果为空可能是视觉服务异常
    if len(results) == 0:
        self.logger.error("视觉识别调用服务异常")
        raise XYZExceptionBase(error_code="E0609",
                    error_msg="")                  

    
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
    self.logger.info(results)    
    if len(results)!=1:
        self.logger.error("Attention! There are more than one top box.")
        raise XYZExceptionBase(error_code="E0608",
                    error_msg="")  
                        
    pallet_pose = results[0]["pose"]
    # pallet_pose[2]+=(height-0.15) 
    outputs["pallet_pose"] = pallet_pose
        

    return "success"
