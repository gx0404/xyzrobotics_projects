from __future__ import division
import copy
import time
import numpy as np

from xyz_motion import SE3
from xyz_homogeneous_bin_packing import HomogeneousBinPacking as HBP
from xyz_homogeneous_bin_packing import OrientedBinPacking as OBP
from xyz_depalletize_motion import utils
from rafcon.xyz_exception_base import XYZLayoutException, XYZExceptionBase
from xyz_env_manager.msg import PrimitiveGroup, GeometricPrimitive
from xyz_motion import PlanningEnvironmentRos
from xyz_env_manager.client import get_planning_environment, set_planned_items, clear_container_all_items
from xyz_env_manager.client import add_container_items, add_bottom_padding_workspace
from xyz_env_manager.client import get_container_items, get_all_robot_tool_states
from xyz_motion import AllRobotToolStatesRos
from xyz_motion import pose_to_list
from xyz_motion import FormattedPlanBox, FormattedRealBox, SE3


import tf.transformations as tfm

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


def parse_poses_from_hmi(pallet_design, sku_height, mirror_config):
    """Parse the HMI response, rectifying the origin and considering mirror config.

    Args:
        pallet_design (tuple[list[xyz_quat], list[xyz_quat]]): layout response from HMI.
        sku_height (float): SKU height.
        mirror_config (Literal["parallel", "vertical", "horizontal", "rotation"]):
            Relationship between the odd and even layers.

    Raises:
        Exception: Invalid mirror config.

    Returns:
        n_boxes (int): number of boxes in a layer.
        odd_poses (list[xyz_quat]): final poses of boxes in an odd layer.
        even_poses (list[xyz_quat]): final poses of boxes in an even layer.
    """
    odd_poses = np.array(pallet_design[0])
    odd_poses[:, :3] /= 1000.0
    odd_poses[:, 2] += sku_height  # HMI returns bottom center, while we'll use top center
    even_poses = np.array(pallet_design[1])
    even_poses[:, :3] /= 1000.0
    even_poses[:, 2] += sku_height
    n_boxes = len(odd_poses)

    if mirror_config == u"rotation":
        even_poses *= [-1, -1, 1, 1, 1, 1, 1]
    elif mirror_config == u"vertical":
        even_poses *= [1, -1, 1, 1, 1, 1, 1]
    elif mirror_config == u"horizontal":
        even_poses *= [-1, 1, 1, 1, 1, 1, 1]
    elif mirror_config == u"parallel":
        pass
    else:
        # same behavior as solver.generate_both_layouts() getting invalid mirror code
        raise Exception("Unrecognized mirror code.")

    return n_boxes, odd_poses, even_poses


def execute(self, inputs, outputs, gvm):
    """
    This state provides mocked vision data.

    Args:
        Inputs Data:
            scan_code (Optional[int]): Default value (None).
                Barcode used to query for layout from HMI.
                You can omit it if mock_from_hmi["get_result_by_scan_code"] is set to False.

            sku_info (dict): Default value (None).
                SKU info used in pallet planning.

        Outputs Data:
            None.

        Gvm:
            id_offset (int) {Get/Set}: 0
                Box ID to be assigned to vision results.

    Properties Data:
        conveyer_config (dict):
            Conveyor case only.

            barcode_point_to (Literal): Default value ("+x").
                Define the orientation of barcode w.r.t. the conveyor workspace.
                Assumed to be "+x", "-x", "+y" or "-y".

            conveyor_box_alignment (Literal): Default value ("lower_right").
                Define where to put the box in the conveyor workspace.
                Assumed to be:
                    "upper_left": Aligned to the upper left corner.
                    "upper_right": Aligned to the upper right corner.
                    "lower_right": Aligned to the lower right corner.
                    "lower_left": Aligned to the lower left corner.
                    "center": Set to the center of the workspace.
                    "customized": Use a custom pose.
                    "endflange": Use poses calculated from the endflange poses.

            customized_poses (dict):
                The custom pose. Currently only one custom pose is supported.
                Will be used if conveyer_config["conveyor_box_alignment"] is set to "customized".

            endflange_poses (list[xyz_quat]): Default value ([[]]).
                The custom endflange poses. The top center of box is set to coincide with the origin of the first tip.
                Will be used if conveyer_config["conveyor_box_alignment"] is set to "endflange".

            sku_num (int): Default value (1).
                Number of boxes on the conveyor.

        mix_depalletize_minimal_sku_height (float): Default value (0.0).
            (Not used in this state 2022/8/25)
            Keep it 0.0 to make everything work as expected.

        mock_from_hmi (dict):
            Pallet case only. Define whether to query for layout from HMI and use it as the mock result.
            The two options are incompatible with each other.

            get_result_by_scan_code (bool): Default value (False).
                Query for layout from HMI using the barcode.

            get_result_by_size (bool): Default value (False).
                Query for layout from HMI using the SKU size.

        pallet_config (dict):
            Pallet case only.

            expected_layer (int): Default value (-1).
                Number of layers expected to be mocked.
                Values with special meanings:
                    -1: Maximum possible value, i.e. a full pallet.
                    0: Return "empty" instantly.

            flip_x (bool): Default value (False).
                Invert the X axis of the whole layout, i.e. reflect across the Y axis.

            flip_y (bool): Default value (False).
                Invert the Y axis of the whole layout, i.e. reflect across the X axis.

            guillotine_packing (bool): Default value (False).
                Generate a guillotine layout.

            long_edge_outside (bool): Default value (False).
                (Not used in this state 2022/8/25)

            mirror (Literal): Default value ("rotation").
                Define the relationship between the odd and even layers.
                Assumed to be:
                    "parallel": Identical.
                    "vertical": Symmetric w.r.t. the X axis.
                    "horizontal": Symmetric w.r.t. the Y axis.
                    "rotation": Central symmetric w.r.t the origin.

        robot_id (str): Default value ("0").
            The robot ID.

        shrink_size (float): Default value (0.001).
            Shrink size of each box.

        workspace_id (str): Default value ("1").
            Workspace ID to call vision service.

    Raises:
        Exception: Invalid barcode direction setting.
        XYZLayoutException: [E0900] No box pose can be returned. Please check the pallet dimensions and the box dimensions.
        Exception: Cannot get result by barcode and size from HMI simultaneously.
        XYZExceptionBase: [60007] Pallet design from HMI is empty.
        Exception: Number of layers expected is greater than the maximum possible value, i.e. taller than a full pallet.
        XYZExceptionBase: [E0707] Appointed workspace does not exist in environment.

    Outcomes:
        0: success
        -1: aborted
        -2: preempted
    """
    self.logger.info("Running {}({})({})".format(self.name, self.unique_id, self.smart_data["comment"]))
    EPSILON_DIMENSION = 1e-4
    # BARCODE DIRECTION
    NO_BARCODE = 0
    BARCODE_ON_TWO_LONG_SIDE = 1
    BARCODE_ON_ONE_LONG_SIDE = 2
    BARCODE_ON_TWO_SHORT_SIDE = 3
    BARCODE_ON_ONE_SHORT_SIDE = 4
    workspace_id = str(self.smart_data["workspace_id"])
    id_offset = gvm.get_variable(f"id_offset_ws_{workspace_id}", per_reference = False, default = 0)
    id_offset = 0

    if not gvm.get_variable("motion_payload", per_reference=True, default=None):
        planning_env_msg = get_planning_environment()
        planning_env = PlanningEnvironmentRos.from_ros_msg(planning_env_msg)
    else:
        last_payload = gvm.get_variable("motion_payload", per_reference=True, default=None)
        planning_env = PlanningEnvironmentRos.from_ros_msg(last_payload["planning_environment"])

    sku_info = inputs["sku_info"]
    if "barcode_direction" not in sku_info.keys():
        self.logger.warning("Notice that barcode direction is not set,using default value 0")
        barcode_direction = 0
    else:
        barcode_direction = sku_info["barcode_direction"]
    
    l = float(sku_info['length'])
    w = float(sku_info['width'])
    h = float(sku_info['height'])
    
    minimal_sku_height = 0.0
    ## default we assume this is single depalletize
    if self.smart_data["mix_depalletize_minimal_sku_height"] == 0.0:
        minimal_sku_height = h
    
    
    workspace_ros = planning_env.get_workspace_ros(workspace_id)
    pallet_pose = workspace_ros.get_bottom_pose().xyz_quat
    global tf_base_space
    tf_base_space = workspace_ros.get_bottom_pose()
    
    L, W, H = workspace_ros.get_dimensions()
    pallet_tf = SE3(pallet_pose).homogeneous
    
    workspace_type = workspace_ros.get_type_string()
    
    shrink_size = self.smart_data["shrink_size"]
    
    conveyor_config = self.smart_data["conveyor_config"]
    pallet_config = self.smart_data["pallet_config"]

    if workspace_type == u"conveyor":        
        alignment = conveyor_config["conveyor_box_alignment"]
        barcode_point_to = conveyor_config["barcode_point_to"]
        sku_num = conveyor_config["sku_num"]
        roat = False
        if barcode_point_to not in ["+x", "-x", "+y", "-y"]:
            self.logger.warning("Notice that barcode point direction incorrect, please choose vallue from [+x, -x, +y, -y](str)")
        if (barcode_point_to == "-y" and barcode_direction in [BARCODE_ON_ONE_SHORT_SIDE, BARCODE_ON_TWO_SHORT_SIDE]) or \
            (barcode_point_to == "+x" and barcode_direction in [BARCODE_ON_ONE_LONG_SIDE, BARCODE_ON_TWO_LONG_SIDE]):
            ws_dimensions = [sku_num*w, l, h]
            sku_quat = [0, 0, 0.707, 0.707]
            roat = True
        elif (barcode_point_to == "+y" and barcode_direction in [BARCODE_ON_ONE_SHORT_SIDE, BARCODE_ON_TWO_SHORT_SIDE]) or \
            (barcode_point_to == "-x" and barcode_direction in [BARCODE_ON_ONE_LONG_SIDE, BARCODE_ON_TWO_LONG_SIDE]):
            ws_dimensions = [sku_num*w, l, h]
            sku_quat = [0, 0, -0.707, 0.707]
            roat = True
        elif (barcode_point_to == "-x" and barcode_direction in [BARCODE_ON_ONE_SHORT_SIDE, BARCODE_ON_TWO_SHORT_SIDE]) or \
            (barcode_point_to == "-y" and barcode_direction in [BARCODE_ON_ONE_LONG_SIDE, BARCODE_ON_TWO_LONG_SIDE]):
            ws_dimensions = [sku_num*l, w, h]
            sku_quat = [0, 0, 0, 1]
            roat = False
        elif (barcode_point_to == "+x" and barcode_direction in [BARCODE_ON_ONE_SHORT_SIDE, BARCODE_ON_TWO_SHORT_SIDE]) or \
            (barcode_point_to == "+y" and barcode_direction in [BARCODE_ON_ONE_LONG_SIDE, BARCODE_ON_TWO_LONG_SIDE]):
            ws_dimensions = [sku_num*l, w, h]
            sku_quat = [0, 0, 1, 0]
            roat = False
        else:
            sku_quat = [0, 0, 0, 1]
            ws_dimensions = [sku_num*l, w, h]
            

        solver = HBP([l, w, h], ws_dimensions)
        n_boxes = solver.solve()
        odd_poses, even_poses = solver.generate_both_layouts(pallet_config["mirror"])
        layer = ((id_offset + 1) // n_boxes) % solver.n_layers + 1
        if layer % 2 == 0:
            layout = odd_poses + (solver.n_layers - layer) * np.array([0, 0, h, 0, 0, 0, 0])
        else:
            layout = even_poses + (solver.n_layers - layer) * np.array([0, 0, h, 0, 0, 0, 0])
        results = []
        layout = np.array(layout)
        
        EPSILON = 1e-3
        layout = layout[layout[:, 0].argsort()]
        far_object = layout[-1]
        if roat:
            x_diff = abs(abs(far_object[0]) - abs(L/2.0-EPSILON - w/2.0))
            y_diff = abs(abs(far_object[1]) - abs(W/2.0-EPSILON - l/2.0))
        else:
            x_diff = abs(abs(far_object[0]) - abs(L/2.0-EPSILON - l/2.0))
            y_diff = abs(abs(far_object[1]) - abs(W/2.0-EPSILON - w/2.0))
        for i in range(int(n_boxes)):
            if alignment == u"upper_left":
                layout[i][0:3] = layout[i][0:3]+[-x_diff, y_diff, 0]
                layout[i][3:7] = sku_quat
            elif alignment == u"upper_right":
                layout[i][0:3] = layout[i][0:3]+[x_diff, y_diff, 0]
                layout[i][3:7] = sku_quat
            elif alignment == u"lower_right":
                layout[i][0:3] = layout[i][0:3]+[x_diff, -y_diff, 0]
                layout[i][3:7] = sku_quat
            elif alignment == u"lower_left":
                layout[i][0:3] = layout[i][0:3]+[-x_diff, -y_diff, 0]
                layout[i][3:7] = sku_quat
            elif alignment == u"center":
                layout[i][0:3] = layout[i][0:3]+[0, 0, 0]
                layout[i][3:7] = sku_quat
            elif alignment == u"customized":
                x_diff = conveyor_config["customized_poses"]["x_offset"]
                y_diff = conveyor_config["customized_poses"]["y_offset"]
                z_diff = conveyor_config["customized_poses"]["z_offset"]
                layout[i][0:3] = layout[i][0:3]+[x_diff, y_diff, z_diff]
                assert len(conveyor_config["customized_poses"]["quaternion"]) == 4
                layout[i][3:7] = conveyor_config["customized_poses"]["quaternion"]
            elif alignment == u"endflange":
                endflange_poses = conveyor_config["endflange_poses"]
                all_robot_states_msg = get_all_robot_tool_states()
                all_robot_states = AllRobotToolStatesRos.from_ros_msg(all_robot_states_msg)
                our_robot = all_robot_states.get_robot_ros(self.smart_data["robot_id"])
                robot_tool_state = all_robot_states.get_robot_tool_state(self.smart_data["robot_id"])
                tool_name = robot_tool_state.tool_name
                our_tool = all_robot_states.get_tool_ros(tool_name)
                tip = our_tool.to_ros_msg().tips[0]
                tf_hand_tip = pose_to_list(tip.tip_pose)*np.array([1, 1, -1, 1, 1, 1, 1])
                tf_hand_tip = SE3(tf_hand_tip).homogeneous
                layout = []
                n_boxes = len(endflange_poses)
                for endflange_pose in endflange_poses:
                    tf_world_hand = SE3(endflange_pose).homogeneous
                    tf_world_tip = tf_world_hand.dot(tf_hand_tip)
                    tf_world_tote = workspace_ros.get_bottom_pose().homogeneous
                    tf_tote_obj = np.linalg.inv(tf_world_tote).dot(tf_world_tip)
                    layout.append(SE3(tf_tote_obj).xyz_quat)
            else:
                self.logger.error("Undefined conveyor_box_alignment code.(未定义的位置选项，请重新选择)")
        for i in range(int(n_boxes)):
            box_tf = np.array(SE3(list(layout[i])).homogeneous)
            pose = list(SE3(np.matmul(pallet_tf, box_tf)).xyz_quat)
            result = { 
                      "id": id_offset + i,
                      "pose": pose ,
                      "dimension": [l - shrink_size + EPSILON_DIMENSION, w - shrink_size + EPSILON_DIMENSION, h + EPSILON_DIMENSION],
                      "name": "box_mock",
                      "score": 100.,
                      "sku_info": inputs["sku_info"],
                      "timestamp": int(time.time())
                    }
            results += [result]

    elif workspace_type == u"pallet":
        flip_x = pallet_config["flip_x"]
        flip_y = pallet_config["flip_y"]
        flip_z = pallet_config["flip_z"]
        if barcode_direction == NO_BARCODE and not pallet_config["guillotine_packing"]:
            solver = HBP([l, w, h], [L, W, H])
        elif barcode_direction in [BARCODE_ON_TWO_LONG_SIDE, BARCODE_ON_ONE_LONG_SIDE]:
            solver = OBP([l, w, h], [L, W, H], False, pallet_config["guillotine_packing"], True)
        elif barcode_direction in [BARCODE_ON_TWO_SHORT_SIDE, BARCODE_ON_ONE_SHORT_SIDE]:
            solver = OBP([l, w, h], [L, W, H], False, pallet_config["guillotine_packing"], False)
        else:
            raise Exception("The solver setting is incorrect, Please check your setting! (参数设置不对， 请检查参数设置， 找到对应垛型需要设置的条码朝向！)")       
        n_boxes = solver.solve()
        if solver.n_layers == 0 or n_boxes <= 0:
            raise XYZLayoutException(error_code = "E0900",
                                     error_msg = "No box pose can be returned. Please check the pallet dimensions and the box dimensions.")

        mock_from_hmi = self.smart_data["mock_from_hmi"]
        if mock_from_hmi["get_result_by_size"] and mock_from_hmi["get_result_by_scan_code"]:
            raise Exception("Can not choose more than one mode of mock result from hmi")
        elif mock_from_hmi["get_result_by_size"]:
            from xyz_logistics_hmi_back.utils.design import query_pallet_design_by_size
            pallet_design_data = query_pallet_design_by_size(1000*l, 1000*w, 1000*h)
            if not pallet_design_data:
                raise XYZExceptionBase("60007", "Pallet Design from HMI is empty.")
            n_boxes, odd_poses, even_poses = parse_poses_from_hmi(pallet_design_data["layout"], h, pallet_config["mirror"])
        elif mock_from_hmi["get_result_by_scan_code"]:
            scan_code = inputs["scan_code"]
            from xyz_logistics_hmi_back.utils.design import query_pallet_design_by_scan_code
            pallet_design_data = query_pallet_design_by_scan_code(scan_code)
            if not pallet_design_data:
                raise XYZExceptionBase("60007", "Pallet Design from HMI is empty.")
            n_boxes, odd_poses, even_poses = parse_poses_from_hmi(pallet_design_data["layout"], h, pallet_config["mirror"])
        else:
            odd_poses, even_poses = solver.generate_both_layouts(pallet_config["mirror"])
            odd_poses = np.array(odd_poses)
            even_poses = np.array(even_poses)
        layer = ((id_offset + 1) // n_boxes) % solver.n_layers + 1
        expected_layer = pallet_config["expected_layer"]
        if expected_layer != -1 and id_offset == 0:
            if expected_layer > solver.n_layers:
                raise Exception("期望显示的层数超出工作空间所能规划的")
            layer = solver.n_layers - expected_layer + 1
            id_offset = (layer - 1) * n_boxes
        if (id_offset != 0) and (id_offset % (solver.n_layers * n_boxes) == 0):
            gvm.set_variable(f"id_offset_ws_{workspace_id}", 0) 
            return "empty"
        if layer % 2 == 0:
            layout = odd_poses + (solver.n_layers - layer) * np.array([0, 0, h, 0, 0, 0, 0])
        else:
            layout = even_poses + (solver.n_layers - layer) * np.array([0, 0, h, 0, 0, 0, 0])
        results = []
        if flip_x:
            layout = layout * [-1, 1, 1, 1, 1, 1, 1]
        if flip_y:
            layout = layout * [1, -1, 1, 1, 1, 1, 1]


        for i in range(int(n_boxes)):
            if flip_z:
                box_tf = np.array((SE3(tfm.quaternion_from_euler(*np.deg2rad([0,0,flip_z]).tolist()))*SE3(list(layout[i]))).homogeneous)
            else:    
                box_tf = np.array(SE3(list(layout[i])).homogeneous)
            
            pose = list(SE3(np.matmul(pallet_tf, box_tf)).xyz_quat)
            result = { 
                      "id": id_offset + i,
                      "pose": pose ,
                      "dimension": [l - shrink_size + EPSILON_DIMENSION, w - shrink_size + EPSILON_DIMENSION, h + EPSILON_DIMENSION],
                      "name": "box_mock",
                      "score": 100.,
                      "sku_info": inputs["sku_info"],
                      "timestamp": int(time.time())
                    }
            results += [result]
    else:
        self.logger.error("Undefined workspace type.")
        
    #tf_space_box_real = (tf_base_space.inv())*SE3(pose_to_list(pick_pose.origin))

    def build_primitive(info,pose_dict,pallet_tote_data,pick_tote_data):
        vision_box_proxy = FormattedRealBox(id = int(info["id"])+10, 
                                    tf_world_origin = SE3(info["pose"][0:7]), 
                                    size = info["dimension"],
                                    tf_origin_box_center = SE3([0, 0, -info["dimension"][-1]/2, 0, 0, 0, 1]))
        pg = vision_box_proxy.primitive_group()

        # We only support box object currently
        pg.additional_info.type = "box"
        pg.additional_info.keys.extend(list(map(str, info["sku_info"].keys())) + ["timestamp"])
        pg.additional_info.values.extend(list(map(str, info["sku_info"].values())) + [str(info["timestamp"])])
        pg.additional_info.descriptions = pg.additional_info.keys


        tf_space_box_real = (tf_base_space.inv())*SE3(info["pose"])
        space_box_pose_real = tf_space_box_real.xyz_quat
        self.logger.info(f"拣配视觉料箱坐标XYZ坐标为 {space_box_pose_real[0:3]}")
        for key,values in pose_dict.items():    
            check_list = list(filter(lambda x:abs(space_box_pose_real[x]-values[x])<0.07,[0,1]))  
            check_list_z = list(filter(lambda x:abs(space_box_pose_real[x]-values[x])<0.05,[2]))           
            if len(check_list)==2 and check_list_z:
                box_id = key
                break
            else:
                box_id = -1       
        if box_id<0:
            self.logger.error("未能匹配上位置号")
            return False
        else:    
            self.logger.info(f"匹配到位置id为{box_id}")     
        box_id = str(box_id)  
        #添加box_id  
        pg.additional_info.keys.append("box_id")
        pg.additional_info.values.append(box_id)
        #添加box barcode
        if box_id in pallet_tote_data.keys():
            pg.additional_info.keys.append("barcode")
            pg.additional_info.values.append(pallet_tote_data[box_id]["barcode"]) 
        else:
            self.logger.info(f"wcs下发托盘数据缺少位置号{box_id}条码")
            raise f"wcs下发托盘数据缺少位置号{box_id}条码"               

        pg.additional_info.keys.append("to_ws")
        pg.additional_info.values.append("")                                   
        return pg
    
    workspace = planning_env.get_workspace_ros(workspace_id)
    results = modify_XY_axis(results, workspace, barcode_direction)
    init_results = copy.deepcopy(results)
    box_id = int(time.time()/10000000)
    for layer in range(1,3):
        append_results = copy.deepcopy(init_results)
        for i in append_results:
            box_id+=1
            i["id"] = box_id
            i["pose"][2]+=(0.23-0.012)*layer   
            i["timestamp"] = int(time.time()) 
            results.append(i)

    all_pose_dict = gvm.get_variable("all_pose_dict", per_reference=False, default=None)

    pick_tote_data = gvm.get_variable("pick_tote_data", per_reference=False, default=None)

    cache_pallet_tote_data = gvm.get_variable("cache_pallet_tote_data", per_reference=False, default=None)

    if len(results)!=len(cache_pallet_tote_data):
        raise "视觉生成的箱子数量和wcs下发生成箱子数量不一致"
    
    for name,pose_dict in all_pose_dict.items():
        container_items = [build_primitive(info, pose_dict, cache_pallet_tote_data, pick_tote_data) for info in results]
        if False in container_items:
            self.logger.info(f"匹配{name}面位置号失败")
            continue
        else:
            break    

    if False in container_items:
        self.logger.info(f"匹配所有面位置号失败")        

    outputs["direction"] = name
    try:
        add_container_items(workspace_id, container_items)
        add_bottom_padding_workspace(workspace_id, minimal_sku_height)
        if gvm.get_variable("motion_payload", per_reference=True, default=None):
            planning_env.add_container_items(workspace_id, container_items)
            planning_env.add_bottom_padding_workspace(workspace_id, minimal_sku_height)
            last_payload["planning_environment"] = PlanningEnvironmentRos.to_ros_msg(planning_env)
            gvm.set_variable("motion_payload", last_payload, per_reference=True)
    
    ## TODO: need a specific error type from env manager
    except Exception:
        raise XYZExceptionBase(error_code="E0707", error_msg="Appointed workspace does not exist in environment") 
    # outputs["object_infos"] = results
    id_offset += len(results)
    gvm.set_variable(f"id_offset_ws_{workspace_id}", id_offset) 
    
    return "success"