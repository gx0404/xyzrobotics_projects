import time
from copy import copy
import numpy as np

from xyz_motion import SE3
from xyz_homogeneous_bin_packing import HomogeneousBinPacking as HBP
from xyz_homogeneous_bin_packing import OrientedBinPacking as OBP
from xyz_homogeneous_bin_packing import NaiveStabilityChecker as StabilityChecker
from xyz_depalletize_motion import utils
from xyz_depalletize_motion.pallet_box import PalletBox
from rafcon.xyz_exception_base import XYZLayoutException, XYZExceptionBase
from xyz_motion import FormattedPlanBox
from xyz_motion import PlanningEnvironmentRos
from xyz_env_manager.client import get_planning_environment, set_planned_items, clear_container_all_items

def execute(self, inputs, outputs, gvm):
    """
    This state calculates placing box poses in the world frame.

    Args:
        Inputs Data:
            sku_info (Dict): Default value (None).
                Sku info used in pallet planning.
                
        Gvm: 
            None


    Properties Data:
        conveyor_case_config, (dict):
            boxes_layout_shape, (list): Default value ([1,1]).
                The shape of the generated place poses on the conveyor.
                [x,y] stands for x rows and y columns.

            auto_layout_shape, (list): Default value ([False, False]).
                Set to True would automatically compute the number of boxes on that axis.
                The computation is based on the sku dimensions and your workspace dimensions.

            rotate_90_degrees, (bool): Default value (False).
                Conveyor case only. Set to True will rotate the SKU by 90 degrees.
            
        
        mirror, (unicode): Default value (u'mirror').
            The code used to define the relationship between the odd and even layers.
            The code is assumed in [u'parallel', u'horizontal', u'vertical', u'mirror'].

        place_workspace_id, (unicode): Default value (u"1").
            The id of placing workspace

        spacing, (bool): Default value (False).
            Pallet case only. Set to True would remove spacing between boxes in the same block.
        

    Raises:
        XYZLayoutException: No box pose can be returned. Please check the pallet dimensions and the box dimensions.
        XYZLayoutException: Layout stability check failed.
        XYZLayoutException: Place pose could cause part of some box out of the workspace.

    Outcomes:
        0: success
        -1: aborted
        -2: preempted
    """
    self.logger.info("Running {}({})({})".format(self.name, self.unique_id, self.smart_data["comment"]))

    planning_env_msg = get_planning_environment()
    planning_env = PlanningEnvironmentRos.from_ros_msg(planning_env_msg)

    sku_info = inputs["sku_info"]
    l = float(sku_info['length'])
    w = float(sku_info['width'])
    h = float(sku_info['height'])
    # 内嵌型料箱的卡槽高度：
    overlapping_heihgt = 0.0
    if inputs["extra"]:  # inputs["extra"]值的形式：{'extra1': {'key': 'overlapping_heihgt', 'value': '20'}}
        for value_l in inputs["extra"].values():
            if value_l.get('key', '') == 'overlapping_heihgt':
                overlapping_heihgt = float(value_l.get("value", 0.0))/1000.0
                overlapping_heihgt = inputs["sku_info"]["overlapping_heihgt"] 
                self.logger.info("获取到内嵌型料箱卡槽高度为：{}m".format(overlapping_heihgt))
                break
    barcode_direction = sku_info.setdefault('barcode_direction', 0)
    if barcode_direction not in range(5):
        raise Exception("barcode direction should be 0, 1, 2, 3, 4 (条码朝向值应该为 0, 1, 2, 3, 4)")

    workspace_ros = planning_env.get_workspace_ros(str(self.smart_data["place_workspace_id"]))
    pallet_pose = workspace_ros.get_bottom_pose().xyz_quat
    L, W, H = workspace_ros.get_dimensions()
    pallet_tf = SE3(pallet_pose).homogeneous
    if L < l or W < w or H < h:
        raise XYZLayoutException(error_code = "E0999",
                                 error_msg = "The dimension of workspace ({}, {}, {}) is smaller than sku ({}, {}, {})!".format(L, W, H, l, w, h))

    workspace_type = workspace_ros.get_type_string()
    flip = self.smart_data["flip"]
    if workspace_type == u'conveyor':
        conveyor_case_config = self.smart_data['conveyor_case_config']
        barcode_point_to = conveyor_case_config["barcode_point_to"]
        assert barcode_point_to in ["+x", "-x", "+y", "-y"]
        boxes_shape = conveyor_case_config['boxes_layout_shape']
        auto_shape = np.array(conveyor_case_config['auto_layout_shape'])
        if barcode_direction == 0:
            box_dims = [l, w, h]
            box_pose = [[0, 0, h, 0, 0, 0, 1]]
        elif (barcode_direction in [3, 4] and barcode_point_to == "+x") or (barcode_direction in [1, 2] and barcode_point_to == "+y"):
            box_dims = [l, w, h]
            box_pose = [[0-0.05, 0, h+0.015, 0, 0, 1, 0]]   
            # box_pose = [[0, 0, h, 0, 0, 1, 0]]
        elif (barcode_direction in [3, 4] and barcode_point_to == "-x") or (barcode_direction in [1, 2] and barcode_point_to == "-y"):
            box_dims = [l, w, h]
            box_pose = [[0, 0, h, 0, 0, 0, 1]]
        elif (barcode_direction in [3, 4] and barcode_point_to == "-y") or (barcode_direction in [1, 2] and barcode_point_to == "+x"):
            box_dims = [w, l, h]
            box_pose = [[(L-l)/2, 0, h, 0, 0, 0.7071, 0.7071]]
        elif (barcode_direction in [3, 4] and barcode_point_to == "+y") or (barcode_direction in [1, 2] and barcode_point_to == "-x"):
            box_dims = [w, l, h]
            box_pose = [[0, 0, h, 0, 0, -0.7071, 0.7071]]
        else:
            raise XYZLayoutException(error_code = "E0999",
                                     error_msg = "Invalid config.")
        boxes_shape = (np.floor([W / box_dims[1], L / box_dims[0]]) * auto_shape +
                       boxes_shape * np.logical_not(auto_shape)).astype(int).tolist()
        boxes_size = np.array(boxes_shape[::-1]) * box_dims[:2]

        if np.any(np.greater(boxes_size, [L, W])):
           raise XYZLayoutException(error_code = "E0902",
                                    error_msg = "Place pose could cause part of some box out of the workspace.")

        box_poses = []
        for i in range(boxes_shape[0]):
            for j in range(boxes_shape[1]):
                transform = [-boxes_size[0] / 2. + box_dims[0] * (j + 0.5), - boxes_size[1] / 2. + box_dims[1] * (i + 0.5), 0, 0, 0, 0, 0]
                box_poses.append([list(np.array(pose) + transform) for pose in box_pose])
        place_poses = []
        for poses in box_poses:
            place_pose = [list(SE3(np.matmul(pallet_tf, SE3(p).homogeneous)).xyz_quat) for p in poses]
            place_poses += [place_pose]
    
    elif workspace_type == u'pallet':
        if barcode_direction == 0:
            solver = HBP([l, w, h], [L, W, H], spacing = self.smart_data["spacing"])
        elif barcode_direction in [1, 2]:
            solver = OBP([l, w, h], [L, W, H], spacing = self.smart_data["spacing"], guillotine_packing = self.smart_data["guillotine_packing"], long_edge_outside = True)
        elif barcode_direction in [3, 4]:
            solver = OBP([l, w, h], [L, W, H], spacing = self.smart_data["spacing"], guillotine_packing = self.smart_data["guillotine_packing"], long_edge_outside = False)
        n = solver.solve()
        if solver.n_layers == 0 or n <= 0:
            raise XYZLayoutException(error_code = "E0900",
                                    error_msg = "No box pose can be returned. Please check the pallet dimensions and the box dimensions.")

        palletize_layers = solver.n_layers
        enable_place_pose_from_hmi = self.smart_data['enable_place_pose_from_hmi']
        if enable_place_pose_from_hmi['place_pose_by_size'] and enable_place_pose_from_hmi['place_pose_by_scan_code']:
            raise Exception("Can not choose more than one mode of place pose from hmi")
        odd_layer_drop_buffer = []
        even_layer_drop_buffer = []
        if enable_place_pose_from_hmi['place_pose_by_size']:
            from xyz_logistics_hmi_back.utils.design import query_pallet_design_by_size
            pallet_design_data = query_pallet_design_by_size(1000*l, 1000*w, 1000*h, enable_place_pose_from_hmi['pallet_id'])
            if not pallet_design_data:
                raise XYZExceptionBase("60007", "从HMI获取的垛型规划结果为空，原因：1）可能没有该尺寸sku的垛型规划；2）托盘ID可能输入错误；3）可能存在2个以上相同尺寸sku的垛型规划")
            pallet_design = pallet_design_data["layout"]
            odd_poses = np.array(pallet_design[0])
            odd_poses[:, :3] /= 1000.0
            even_poses = np.array(pallet_design[1])
            even_poses[:, :3] /= 1000.0
            if enable_place_pose_from_hmi["drop_height_from_hmi"]:
                odd_layer_drop_buffer = [op['place_drop_buffer'] for op in pallet_design_data["objects"]["odd_layer"]]
                even_layer_drop_buffer = [ep['place_drop_buffer'] for ep in pallet_design_data["objects"]["even_layer"]]
            if enable_place_pose_from_hmi['layers_from_hmi']:
                palletize_layers = pallet_design_data["layers"]
        elif enable_place_pose_from_hmi['place_pose_by_scan_code']:
            scan_code = inputs["scan_code"]
            if self.smart_data["place_workspace_id"] == "2" or self.smart_data["place_workspace_id"] == "3":
                scan_code = gvm.get_variable("cache_scan_code", per_reference=False, default=None)
                hmi_pallet_id = "2"
            else:        
                scan_code = gvm.get_variable("depal_scan_code", per_reference=False, default=None)
                hmi_pallet_id = "1"
            from xyz_logistics_hmi_back.utils.design import query_pallet_design_by_scan_code
            pallet_design_data = query_pallet_design_by_scan_code(scan_code, hmi_pallet_id)
            if not pallet_design_data:
                raise XYZExceptionBase("60007", "从HMI获取的垛型规划结果为空，原因：1）可能没有该条码sku的垛型规划；2）托盘ID可能输入错误")
            pallet_design = pallet_design_data["layout"]
            odd_poses = np.array(pallet_design[0])
            odd_poses[:, :3] /= 1000.0
            even_poses = np.array(pallet_design[1])
            even_poses[:, :3] /= 1000.0
            if enable_place_pose_from_hmi["drop_height_from_hmi"]:
                odd_layer_drop_buffer = [op['place_drop_buffer'] for op in pallet_design_data["objects"]["odd_layer"]]
                even_layer_drop_buffer = [ep['place_drop_buffer'] for ep in pallet_design_data["objects"]["even_layer"]]
            if enable_place_pose_from_hmi['layers_from_hmi']:
                palletize_layers = pallet_design_data["layers"]
        else:
            odd_poses, even_poses = solver.generate_both_layouts(self.smart_data['mirror'])
            if flip != '':
                odd_poses = np.array(odd_poses)
                even_poses = np.array(even_poses)
                if flip in ['x', 'X']:
                    if barcode_direction == 0:
                        odd_poses = odd_poses * [-1, 1, 1, 1, 1, 1, 1]
                        even_poses = even_poses * [-1, 1, 1, 1, 1, 1, 1]
                    else:
                        odd_poses, even_poses = solver.flip(flip)
                elif flip in ['y', 'Y']:
                    if barcode_direction == 0:
                        odd_poses = odd_poses * [1, -1, 1, 1, 1, 1, 1]
                        even_poses = even_poses * [1, -1, 1, 1, 1, 1, 1]
                    else:
                        odd_poses, even_poses = solver.flip(flip)
                else:
                    raise Exception("错误的flip代码，请在(x, X, y, Y)中选择")

        sc = StabilityChecker(odd_poses, even_poses, [l, w, h], [L, W, H])
        stable = sc.stability_check()
        if stable is False:
            raise XYZLayoutException(error_code = "E0901",
                                    error_msg = "Layout stability check failed.")


        dropped_poses = []
        drop_buffer_list = []
        place_id = str(self.smart_data["place_workspace_id"])
        first_offset = False
        for i in range(1, palletize_layers+1, 1):
            if i % 2 == 1:            
                if place_id in ["2","3"] and i==3:
                    pass
                    #import ipdb;ipdb.set_trace()  
                    odd_poses[:, 1] -=0.003    
                odd_poses[:, 2] = h + (i-1) * (h - overlapping_heihgt)
                odd_tf = np.array([SE3(p).homogeneous for p in odd_poses])
                
                tf_base = np.matmul(pallet_tf, odd_tf)
                if odd_layer_drop_buffer:
                    drop_buffer_list.extend(odd_layer_drop_buffer)
            else:
                if place_id in ["2","3"] and i==2:
                    pass
                    #import ipdb;ipdb.set_trace()   
                    even_poses[:, 1] -=0.003              
                even_poses[:, 2] = h + (i-1) * (h - overlapping_heihgt)
                even_tf = np.array([SE3(p).homogeneous for p in even_poses])
                tf_base = np.matmul(pallet_tf, even_tf)
                if even_layer_drop_buffer:
                    drop_buffer_list.extend(even_layer_drop_buffer)
            poses_base = [list(SE3(m).xyz_quat) for m in tf_base]
            boxes = []
            for j in range(len(poses_base)):
                boxes.append(
                    PalletBox(poses_base[j] +\
                            [l, w, h] +\
                            [0] +\
                            [0]))
            for j in range(len(boxes)):
                box2list = lambda box: [box.x, box.y, box.z, box.qx, box.qy, box.qz, box.qw]
                dropped_poses.append([box2list(boxes[j])])

        place_poses = copy(dropped_poses)
    else:
        self.logger.error("UnSupport workspace type {}".format(workspace_type))

    items = []
    for i in range(len(place_poses)):
        plan_box_proxy = FormattedPlanBox(id = i, 
                                      tf_world_origin = SE3(place_poses[i][0][0:7]), 
                                      size = [l, w, h], 
                                      tf_origin_box_center = SE3([0, 0, -h/2.0, 0, 0, 0, 1]))
        item = plan_box_proxy.primitive_group()

        # We only support box object currently
        item.additional_info.type = "box"
        item.additional_info.keys.extend(list(map(str, sku_info.keys())) + ["timestamp"])
        item.additional_info.values.extend(list(map(str, sku_info.values())) + [str(time.time())])
        if workspace_type == u'pallet' and drop_buffer_list:
            item.additional_info.keys.extend(["place_drop_buffer"])
            item.additional_info.values.extend([str(drop_buffer_list[i])])
        item.additional_info.descriptions = item.additional_info.keys

        items.append(item)

    ## TODO Do we need sync environment always in palletize calc place pose.
    if gvm.get_variable("motion_payload", per_reference=True, default=None):
        ## we sync environment
        self.logger.error("The motion_payload should be clear, this can be a bug")
        if self.smart_data["clear_motion_payload"]:
            gvm.set_variable("motion_payload", None, per_reference=True)
        else:
            last_payload = gvm.get_variable("motion_payload", per_reference=True, default=None)
            planning_env = PlanningEnvironmentRos.from_ros_msg(last_payload["planning_environment"])
            planning_env.set_planned_items(self.smart_data["place_workspace_id"], items, [])
            last_payload["planning_environment"] = PlanningEnvironmentRos.to_ros_msg(planning_env)
            gvm.set_variable("motion_payload", last_payload, per_reference=True)
            
    set_planned_items(self.smart_data["place_workspace_id"], items, [])
    
    return "success"

