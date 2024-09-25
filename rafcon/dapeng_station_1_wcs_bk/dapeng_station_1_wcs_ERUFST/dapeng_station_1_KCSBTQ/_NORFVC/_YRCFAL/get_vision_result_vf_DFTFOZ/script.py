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
    vision_service = "calculate_object_poses_collision"


    if flag_capture_image:
        start_time = time.time()
        eye_in_hand = self.smart_data["eye_in_hand"]  
        while not self.preempted:
            if eye_in_hand:
                r = RobotDriver(0)
                scan_pose = r.get_cartpose().xyz_quat
                capture_res = vision_bridge.run(int(vision_id), "capture_images_1",info=json.dumps({"tf_world_hand":scan_pose}))
            else:
                capture_res = vision_bridge.run(int(vision_id), "capture_images_1")                  
            if gvm.variable_exist("ERROR"):
                return "empty"
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
        
    vision_result_raw = vision_bridge.run(int(vision_id), vision_service)
    height = vision_result_raw.info
    self.logger.info(f"height is {height}")
    if float(height)<0.2:
        return "empty"
    
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
                # sku_info["length"],
                # sku_info["width"],
                # sku_info["height"],
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

    if len(results) == 1:
        outputs["collision_pose"] = results[0]["pose"]  
        return "success"         
     
    else:        
        return "empty"     

