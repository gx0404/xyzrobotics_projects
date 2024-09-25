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
    
    sku_info = inputs["sku_info"]
    sku_dimension = [sku_info["length"],sku_info["width"],sku_info["height"]]
    sku_dimension = list(map(lambda x:round(x,3),sku_dimension))
    
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
    sku_dimension = [sku_info["length"],sku_info["width"],sku_info["height"]]
    sku_dimension = list(map(lambda x:round(x,3),sku_dimension))
    
    if sku_dimension==[0.4,0.3,0.23]:
        vision_service = "calculate_object_poses_1_0"
    elif sku_dimension==[0.6,0.4,0.23]:
        vision_service = "calculate_object_poses_2_0"
    else:
        raise "无效的尺寸"      
    
    if mode=="RELEASE" or (mode=="DEBUG" and data_source_path==""):
        capture_service_list = ["capture_images_to_hand_"+str(i) for i in range(4)]
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
                            merge_ftr = vision_bridge.async_run(int(vision_id), "merge_cloud_"+str(index-2)) 
                            #vision_bridge.run(int(vision_id), "merge_cloud_"+str(index-1)) 
                            self.logger.info(f"调用合并service: "+"merge_cloud_"+str(index-2))
                            merge_ftr.get()
                        break
                    except Exception as e:
                        self.logger.info(f"拍照service: {capture_service},异步拍照结果还未出现,重新获取")
                        self.logger.info(f"{e}")
                        self.preemptive_wait(0.1)
                        continue
                
                
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
                    
                    
        merge_ftr = vision_bridge.async_run(int(vision_id), "merge_cloud_2") 
        #vision_bridge.run(int(vision_id), "merge_cloud_2") 
        self.logger.info(f"调用合并service: merge_cloud_2")  
        merge_ftr.get() 
        ftr = vision_bridge.async_run(int(vision_id),vision_service)    
        #vision_result_raw = vision_bridge.run(int(vision_id),vision_service) 

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
            return "empty"                    

        
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
        global send_error_msg
        send_error_msg = ""       
        f = open('/home/xyz/xyz_app/app/rafcon/check_pallet_tote_data.log', 'w')  
                       
        def build_primitive(info,pose_dict,pallet_tote_data,pick_tote_data,f):
            global send_error_msg
            vision_box_proxy = FormattedRealBox(id = int(info["id"]), 
                                        tf_world_origin = SE3(info["pose"][0:7]), 
                                        size = info["dimension"],
                                        tf_origin_box_center = SE3([0, 0, -info["dimension"][-1]/2, 0, 0, 0, 1]))
            pg = vision_box_proxy.primitive_group()
            
            # We only support box object currently
            pg.additional_info.type = "box"
            pg.additional_info.keys.extend(list(map(str, info["sku_info"].keys())) \
                + ["timestamp", "vision_id"])
            pg.additional_info.values.extend(list(map(str, info["sku_info"].values())) \
                + [str(info["timestamp"]), str(info["vision_id"])])
            pg.additional_info.descriptions = pg.additional_info.keys
            
            tf_space_box_real = (tf_base_space.inv())*SE3(info["pose"])
            space_box_pose_real = tf_space_box_real.xyz_quat
            f.write(f"\n拣配视觉料箱坐标XYZ坐标为 {space_box_pose_real[0:3]}")
            
            for key,values in pose_dict.items():    
                check_list = list(filter(lambda x:abs(space_box_pose_real[x]-values[x])<0.08,[0,1]))  
                check_list_z = list(filter(lambda x:abs(space_box_pose_real[x]-values[x])<0.05,[2]))           
                if len(check_list)==2 and check_list_z:
                    box_id = key
                    break
                else:
                    box_id = -1       
            if box_id<0:
                f.write("\n未能匹配上位置号")
                send_error_msg+=f"\n拣配视觉料箱坐标{space_box_pose_real[0:3]}未能匹配上位置号,"
                return False
            else:    
                f.write(f"\n匹配到位置id为{box_id}")     
                send_error_msg+=f"\n匹配到位置id为{box_id},"
            box_id = str(box_id)  
            #添加box_id  
            pg.additional_info.keys.append("box_id")
            pg.additional_info.values.append(box_id)
            #添加box barcode
            if box_id in pallet_tote_data.keys():
                pg.additional_info.keys.append("barcode")
                pg.additional_info.values.append(pallet_tote_data[box_id]["barcode"]) 
            else:
                f.write(f"\nwcs下发托盘数据缺少位置号{box_id}条码")
                send_error_msg+=f"\nwcs下发托盘数据缺少位置号{box_id}条码,"
                return False              
            if str(box_id) in pick_tote_data.keys():
                pg.additional_info.keys.append("to_ws")
                pg.additional_info.values.append(pick_tote_data[box_id]["to_ws"])
                pg.color.r = 1
                pg.color.g = 1
                pg.color.b = 1
                pg.color.b = 1
            else:
                pg.additional_info.keys.append("to_ws")
                pg.additional_info.values.append("1")                                   
            return pg
        
        all_pose_dict = gvm.get_variable("all_pose_dict", per_reference=False, default=None)

        pick_tote_data = gvm.get_variable("pick_tote_data", per_reference=False, default=None)

        pallet_tote_data = gvm.get_variable("pallet_tote_data", per_reference=False, default=None)

        init_results = copy.deepcopy(results)
        box_id = int(time.time()/10000000)    
        overlapping_heihgt = inputs["sku_info"]["overlapping_heihgt"]    
        
        for result in init_results:
            tf_space_box_real = (tf_base_space.inv())*SE3(result["pose"])
            layer = round(tf_space_box_real.xyz_quat[2]/(sku_info["height"]-overlapping_heihgt))
            self.logger.info(f"视觉pose为{result['pose']},当前层数为{layer}")
            f.write(f"视觉pose为{result['pose']},当前层数为{layer}\n")
            for i in range(1,layer): 
                append_result = copy.deepcopy(result)    
                box_id+=1          
                append_result["id"] = box_id
                append_result["pose"][2]-=i*(sku_info["height"]-overlapping_heihgt)
                append_result["timestamp"] = int(time.time())
                results.append(append_result)

        def build_primitive_planning_wcs_env(pallet_tote_data,all_pose_dict,row):
            pallet_box_dict = all_pose_dict["C"]
            if row==5:
                rotate = {
                    1:[0,0,0.707,0.707],
                    2:[0,0,0.707,0.707],
                    3:[0,0,0.707,0.707],
                    4:[0,0,0,1],
                    5:[0,0,0,1]
                }
                dimension = [0.6,0.4,0.23]
            elif row==9:
                rotate = {
                    1:[0,0,0.707,0.707],
                    2:[0,0,0.707,0.707],
                    3:[0,0,0.707,0.707],
                    4:[0,0,0.707,0.707],
                    5:[0,0,0,1],
                    6:[0,0,0,1],
                    7:[0,0,0.707,0.707],
                    8:[0,0,0,1],
                    9:[0,0,0,1],
                }  
                dimension = [0.4,0.3,0.23]     
            container_items = []               
            for box_id,pose_xyz in pallet_box_dict.items():
                if str(box_id) in pallet_tote_data.keys():
                    row_id = box_id%row
                    if row_id == 0:
                        row_id = row    
                        
                    pose_rotate = rotate[row_id] 
                    tf_space_obj = SE3(pose_xyz+pose_rotate)
                    tf_base_obj = tf_base_space*tf_space_obj 
                    
                    vision_box_proxy = FormattedRealBox(id = box_id, 
                                                tf_world_origin = tf_base_obj, 
                                                size = dimension,
                                                tf_origin_box_center = SE3([0, 0, -dimension[-1]/2, 0, 0, 0, 1]))
                    pg = vision_box_proxy.primitive_group()


                    # We only support box object currently
                    pg.additional_info.type = "box"                             
                    container_items.append(pg)  
            return container_items   
        current_time = time.time()         
        row = gvm.get_variable("row", per_reference=False, default=None)
        wcs_container_items =build_primitive_planning_wcs_env(pallet_tote_data,all_pose_dict,row)
        
        
        wcs_planning_env = copy.deepcopy(planning_env)
        for clear_id in ["0","1","2","3","4","5","6"]:           
            wcs_planning_env.remove_bottom_padding_workspace(clear_id)
            wcs_planning_env.clear_container_all_items(clear_id)
            wcs_planning_env.remove_full_padding_workspace(clear_id)
            wcs_planning_env.remove_bottom_padding_workspace(clear_id)
            
        wcs_planning_env.add_container_items(workspace_id, wcs_container_items)
        wcs_planning_env.to_json_file("/home/xyz/xyz_app/projects/dapeng_station_1/wcs_pallet_json/planning_env.json")
        self.logger.info(f"load time is {time.time()-current_time}")
                
        from xyz_logistics_hmi_back.utils.utils import send_order_log
                       
        if len(results)!=len(pallet_tote_data):
            send_error_msg = f"视觉生成的箱子数量和wcs下发生成箱子数量不一致"
            send_order_log(message=send_error_msg, status=False) 
            error_info = {
                "error": '10000',
                "data":{
                "error_msg": send_error_msg,
                "zh_msg": "",
                "error_code": '10000',
                "tip": ""
                }
            }
            outputs["error_info"] = error_info
            self.logger.info(f"视觉生成的箱子数量和wcs下发生成箱子数量不一致")
            return "fault_task"
            raise "视觉生成的箱子数量和wcs下发生成箱子数量不一致"
        
        if not inputs["current_direction"]:
            current_direction = "C"
        else:
            current_direction = inputs["current_direction"]   
                    
        check_c_flag = False
        for name,pose_dict in all_pose_dict.items():
            container_items = [build_primitive(info, pose_dict, pallet_tote_data, pick_tote_data,f) for info in results]
            if False in container_items:
                if name == current_direction:
                    if send_error_msg.find("未能匹配上位置号")>0:
                        self.logger.info(f"{current_direction}面视觉匹配失败")
                        check_c_flag = True
                    else:    
                        self.logger.info(f"匹配{name}面位置号失败")
                        send_error_msg = f""
                else: 
                    self.logger.info(f"匹配{name}面位置号失败")
                    send_error_msg = f""                            
                continue
            else:
                break    

        if False in container_items:
            self.logger.info(f"匹配所有面位置号失败")  
            if check_c_flag:
                send_error_msg = f"视觉匹配失败,请检查摆正垛型,检查视觉匹配结果"
            else:
                send_error_msg = f"匹配所有面位置号失败,上游下发虚拟储位异常"    
            send_order_log(message=send_error_msg, status=False)          
            error_info = {
                "error": '10000',
                "data":{
                "error_msg": send_error_msg,
                "zh_msg": "",
                "error_code": '10000',
                "tip": ""
                }
            }
            outputs["error_info"] = error_info
            return "fault_task"
            raise XYZExceptionBase("20010", "匹配所有面位置号失败") 

        outputs["direction"] = name
     
        if name != current_direction:
            self.logger.info(f"实际转向不为{current_direction}面")   
            send_error_msg = f"实际转向不为{current_direction}面"
            send_order_log(message=send_error_msg, status=False) 
            error_info = {
                "error": '10000',
                "data":{
                "error_msg": send_error_msg,
                "zh_msg": "",
                "error_code": '10000',
                "tip": ""
                }
            }
            outputs["error_info"] = error_info      
            return "fault_task"         
            raise "实际转向不为C面"        
        try:
            clear_container_all_items(workspace_id)
            self.logger.warning(f"清除真实环境中工作空间{workspace_id}的原有物体")
            add_container_items(workspace_id, container_items)
            add_bottom_padding_workspace(workspace_id, minimal_sku_height)
            if gvm.get_variable("motion_payload", per_reference=True, default=None):
                self.logger.warning(f"清除规划环境中工作空间{workspace_id}的原有物体")
                planning_env.clear_container_all_items(workspace_id)
                planning_env.add_container_items(workspace_id, container_items)
                planning_env.add_bottom_padding_workspace(workspace_id, minimal_sku_height)
                last_payload["planning_environment"] = PlanningEnvironmentRos.to_ros_msg(planning_env)
                gvm.set_variable("motion_payload", last_payload, per_reference=True)
        ## TODO: need a specific error type from env manager
        except Exception:
            raise XYZExceptionBase(error_code="E0707", error_msg="Appointed workspace does not exist in environment")
        if mode=="DEBUG" and data_source_path=="":
            if not os.path.exists(DEFAULT_OFFLINE_VISION_PATH):
                os.mkdir(DEFAULT_OFFLINE_VISION_PATH)
            cameras = '_'.join(camera_ids)
            file_name = "ws_{}_{}_{}.json".format(vision_id, ts, cameras)
            planning_env_msg = get_planning_environment()
            planning_env = PlanningEnvironmentRos.from_ros_msg(planning_env_msg)
            data_store_path = os.path.join(DEFAULT_OFFLINE_VISION_PATH, file_name)
            pick_workspace = planning_env.get_workspace_ros(str(vision_id))
            pick_workspace.to_json_file(data_store_path)

    elif mode=="DEBUG" and data_source_path!="":
        if '/' not in data_source_path:
            data_path = os.path.join(DEFAULT_OFFLINE_VISION_PATH, data_source_path)
        else:
            data_path = data_source_path
        if not os.path.exists(data_path):
            raise Exception("incorrect path(路径设置错误，请检查)")
        pick_workspace = WorkspaceRos.from_json_file(data_path)
        container_items = pick_workspace.get_container_items()
        try:
            clear_container_all_items(workspace_id)
            self.logger.warning(f"清除真实环境中工作空间{workspace_id}的原有物体")
            add_container_items(workspace_id, container_items)
            add_bottom_padding_workspace(workspace_id, minimal_sku_height)
            if gvm.get_variable("motion_payload", per_reference=True, default=None):
                planning_env.clear_container_all_items(workspace_id)
                self.logger.warning(f"清除规划环境中工作空间{workspace_id}的原有物体")
                planning_env.add_container_items(workspace_id, container_items)
                planning_env.add_bottom_padding_workspace(workspace_id, minimal_sku_height)
                last_payload["planning_environment"] = PlanningEnvironmentRos.to_ros_msg(planning_env)
                gvm.set_variable("motion_payload", last_payload, per_reference=True)
        ## TODO: need a specific error type from env manager
        except Exception:
            raise XYZExceptionBase(error_code="E0707", error_msg="Appointed workspace does not exist in environment")
    else:
        raise Exception("mode or file path setting incorrect（模式或者路径设置错误， 请检查!）")
    if self.smart_data["flag_check_primitives_change"]:
        try:
            check_change = getattr(vision_bridge, 'check_primitives_change')(vision_id, [])
        except:
            raise Exception("Make sure XVF service is set up corrected.（请检查XVF服务设置!）")
    return "success"
