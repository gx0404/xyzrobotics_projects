from __future__ import print_function, division

import copy
import math
import re
import numpy as np
import tf.transformations as tfm
import shapely.geometry as sg
from multiprocessing import Value

from xyz_motion import SE3
from xyz_motion import ToolRos, AllRobotToolStatesRos, pose_to_list
from xyz_motion import PlanningEnvironmentRos
from xyz_env_manager.msg import GeometricPrimitive, GraspPlan, Pose
from xyz_depalletize_motion import utils
from xyz_depalletize_motion.pallet_box import PalletBox
from xyz_env_manager.client import get_planning_environment, get_all_robot_tool_states
from xyz_env_manager.client import get_robot_base_transform
from xyz_motion import GraspPlanGenerator
from xyz_env_manager.msg import GraspPlanConfig

global debug_fastforward

class _DebugFastforward(Exception):
    """
    An internal exception used to jump out of useless for-loops when user choose to stop the debug show.
    DO NOT FORGET TO CATCH IT.
    """
    pass

def debugshow(chosen_boxes, remaining_boxes, tip, title="Debug"):
    global debug_fastforward

    if debug_fastforward.value:
        raise _DebugFastforward()

    def draw_debug_figure(debug_fastforward):

        def on_press(event):
            if event.key == 'q':
                """
                Exceptions raised here will be caught by matplotlib's default exception handler,
                and I haven't found an elegant way to bind my custom handler.
                So debug_fastforward is still responsible for really raising the exception in the next for-loop.
                """
                debug_fastforward.value = True
                plt.close()
            if event.key == 'n':
                # Clear the current figure
                plt.clf()

        import matplotlib

        import rafcon
        if rafcon.__version__ < '1.2.14':
            # Legacy usage of TkAgg
            matplotlib.use('TkAgg',warn=False, force=True)

        import matplotlib.pyplot as plt
        ax = plt.gca()
        ax.set_title(f"{title} \n 按 n 看下一个；按 q 退出")

        if rafcon.__version__ < '1.2.14':
            # use old light theme
            ax.add_patch(plt.Circle((0.0, 0.0), 0.1, color='#22bbf0', alpha=0.8))
            [box.get_visualization(ax, color='red') for box in chosen_boxes]
            [box.get_visualization(ax, color='blue') for box in remaining_boxes]
            [box.get_visualization(ax, color='green') for box in tip]
        else:
            # else use dark theme by default
            [box.get_visualization(ax, color='#ff5945') for box in chosen_boxes]
            [box.get_visualization(ax, color='#00b0ff') for box in remaining_boxes]
            [box.get_visualization(ax, color='#b2ff59') for box in tip]
            for patch in ax.patches:
                patch.set_alpha(0.7)

            ax.add_patch(plt.Circle((0.0, 0.0), 0.1, color='#22bbf0', alpha=0.8))

            ax.set_aspect('equal')
            multiplier = 1.05
            for axis, setter in [(ax.xaxis, ax.set_xlim), (ax.yaxis, ax.set_ylim)]:
                vmin, vmax = axis.get_data_interval()
                vmin = multiplier * np.floor(vmin / multiplier)
                vmax = multiplier * np.ceil(vmax / multiplier)
                setter([vmin, vmax])

            from matplotlib.ticker import MaxNLocator
            ax.xaxis.set_major_locator(MaxNLocator(nbins = 5, steps=[1, 2, 5, 10]))
            xmin, xmax = ax.xaxis.get_data_interval()
            ymin, ymax = ax.yaxis.get_data_interval()
            ybins = int((ymax - ymin) / float(xmax - xmin)) * 5 + 1
            ax.yaxis.set_major_locator(MaxNLocator(nbins = ybins, steps=[1, 2, 5, 10]))

            plt.grid(True, which='both', linestyle='--')

        cid = plt.gcf().canvas.mpl_connect('key_press_event', on_press)
        plt.draw()
        plt.show()
        plt.gcf().canvas.mpl_disconnect(cid)

    import rafcon
    if rafcon.__version__ >= '1.2.14':
        draw_debug_figure(debug_fastforward)
    else:
        import multiprocessing
        p = multiprocessing.Process(target=draw_debug_figure, args=(debug_fastforward,))
        p.start()
        p.join()
        
def get_id(name):
    from xyz_motion import FormattedPlanBox
    return FormattedPlanBox.get_unique_id_by_name(name)

def get_pallet_box(pg):
    assert (len(pg.primitives) == 1 and pg.primitives[0].type == GeometricPrimitive.BOX)
    assert (pg.additional_info.type == 'box')
    pose = pose_to_list(pg.origin)
    weight_idx = pg.additional_info.keys.index("weight")
    item_id = get_id(pg.name)

    return PalletBox(pose +\
                list(pg.primitives[0].dimensions) +\
                [float(pg.additional_info.values[weight_idx])] +\
                [item_id])

def execute(self, inputs, outputs, gvm):
    """
    This state fetches object info and generates grasp plans based on picking and placing info.
    This state should be used when high-accurancy picking is required (+-5mm). eg. tote

    Args:
        Inputs Data:
            None

        Outputs Data:
            grasp_plan (xyz_env_manager.msg._GraspPlan.GraspPlan): Default value (None).
                GraspPlan message needed in motion planner.

            object_poses (list): Default value (None).
                List of object infos [x, y, z, qx, qy, qz, qw] to be fed into add_object_poses.

        Gvm:
            generate_tote_grasp_plan&${reuse_tag} (unicode)
                Special global variable used in our reuse feature.

    Properties Data:

        comment, (unicode): Default value (u"generate_tote_grasp_plan").
            The comment of this state. This will show in state's GUI block.

        grasp_symmetry_angle, (int): Default value (180).
            Change in angles while looping throught grasp angles from 0 to 360.
            Provides more grasp plans. 
            
        pick_corner_heuristic, (int): Default value (0).
            The id of picking corner

        pick_workspace_id, (unicode): Default value (u"0").
            The id of picking workspace

        place_corner_id, (int): Default value (0).
            The id of placing corner

        place_workspace_id, (unicode): Default value (u"1").
            The id of placing workspace

        fixed_grasp_pose: 固定抓取姿态

            enable, (bool): Default value (false).
                是否开启固定抓取姿态

            fixed_endflange_poses, (list): Default value ([[]]).
                机器人抓取时固定的法兰姿态列表(相对于robot_baseframe), 可以有多个

            modify_grasp_pose_by_size: 根据sku的尺寸修正固定抓取姿态(仅在抓取位为输送线时生效)

                enable, (bool): Default value (false).
                    是否开启根据sku的尺寸修正固定抓取姿态

                conveyor_box_alignment，(str): Default value ("upper_left")
                    箱子在输送线上的靠边选项，与mock vision模块需要一致

                modify_axis, (list): Default value ([0, 1, 0]).
                    选择输送线的x,y,z三个方向中需要修改的方向，1表示这个方向需要修改，0反之

                object_length_direction，(str): Default value ("y")
                    物体的长边在输送线上的朝向，"x"为短边沿着输送线，"y"为长边沿着输送线

                refer_sku_dimensions, (list): Default value ([]).
                    参考物料的尺寸[长，宽，高]，抓取姿态相对于这个尺寸进行偏移

        reuse_tag, (unicode): Default value (u"test_tag").
            The tag used in our reuse feature.

        robot_id, (unicode): Default value (u"0").
            The id of robot

        symmetry_angle, (int): Default value (180).
            The angle by which when the object is rotated around its z axis,
            the rotated geometry appears to be identical to the original one.

        chosen_tip_names, (list): Default value([]]).
            The list of tip names we want use for depalletize. If chosen_tip_names is empty, we choose all tips by default.

        show_valid_grasp_plans:
            This will display all the valid grasp plans in a gui. 
            
        sort_slots_by_corner_id, (bool): Default value (False)
            This is set to True when you want to
            reorder the placing order by the place_corner_id in a pallet.

        sort_slots_by_human_label, (bool): Default value (True)
            If this is True, then the placing order follows the order by which
            each placing item is added to the planning environment. Especially, one can use
            HMI to manually determine the placing order, which will be imported by the palletize_calc_place_pose module
            when its properties_data (service_name) is set correctly.


    Raises:
        Exception: Pick workspace id not found

        Exception: Place workspace id not found

        Exception: Generate grasp plan failed

    Outcomes:
        3: place_plan_empty
        2: place_workspace_full
        1: vision_exhausted
        0: success
        -1: aborted
        -2: preempted
    """
    self.logger.info("Running {}({})({})".format(self.name, self.unique_id, self.smart_data["comment"]))

    if not gvm.get_variable("motion_payload", per_reference=True, default=None):
       planning_env_msg = get_planning_environment()
       planning_env = PlanningEnvironmentRos.from_ros_msg(planning_env_msg)
    else:
       last_payload = gvm.get_variable("motion_payload", per_reference=True, default=None)
       planning_env = PlanningEnvironmentRos.from_ros_msg(last_payload["planning_environment"])


    # info about the pallet/workspace pending for depalletizing.
    pick_ws_id = self.smart_data["pick_workspace_id"]
    pick_workspace = planning_env.get_workspace_ros(str(pick_ws_id))
    if not pick_workspace:
        raise Exception("Pick workspace id not found")

    pick_corner_heuristic = self.smart_data["pick_corner_heuristic"]
    pick_workspace_vertices = utils.get_box_vertices(
        pick_workspace.get_bottom_pose().xyz_quat + pick_workspace.get_dimensions())

    # Debug settings
    show_valid_grasp_plans = self.smart_data["show_valid_grasp_plans"]
    if show_valid_grasp_plans:
        global debug_fastforward
        debug_fastforward = Value('b', False)
        debug_fastforward.value = False
        
    place_ws_id = self.smart_data["place_workspace_id"]
    place_corner_id = self.smart_data["place_corner_id"]
    place_workspace = planning_env.get_workspace_ros(str(place_ws_id))
    if not place_workspace:
        raise Exception("Place workspace id not found")

    place_workspace_vertices = utils.get_box_vertices(
        place_workspace.get_bottom_pose().xyz_quat + place_workspace.get_dimensions())

    # item on the pallet, pending for depalletizing.
    items = planning_env.get_container_items(pick_ws_id)
    if not items:
        return "vision_exhausted"

    all_robot_states_msg = get_all_robot_tool_states()
    all_robot_states = AllRobotToolStatesRos.from_ros_msg(all_robot_states_msg)
    planning_robot_ros = all_robot_states.get_robot_ros(str(self.smart_data["robot_id"]))

    robot_tool_state = all_robot_states.get_robot_tool_state(str(self.smart_data["robot_id"]))
    tool_name = robot_tool_state.tool_name
    cur_tool = all_robot_states.get_tool_ros(tool_name).to_ros_msg()
    
    tf_map_robot = get_robot_base_transform(str(self.smart_data["robot_id"]))
    tf_map_robot = SE3(pose_to_list(tf_map_robot))

    tool = ToolRos.from_ros_msg(cur_tool)
    tip_map = tool.get_tip_map()

    ## tip_map to dict
    tip_map_dict = {}
    ## the code below is for support chosen_tip_names
    chosen_tip_names = self.smart_data["chosen_tip_names"]
    if chosen_tip_names:
        for chosen_tip_name in chosen_tip_names:
            if chosen_tip_name not in tip_map:
                raise Exception("Can not find chosen_tip_name: {} in env-manager tip_map!".
                                format(chosen_tip_name))
            else:
                tip_map_dict[chosen_tip_name] = tip_map[chosen_tip_name]

        tip_map = tip_map_dict
    else:
        for tip_name in tip_map:
            tip_map_dict[tip_name] = tip_map[tip_name]
        tip_map = tip_map_dict


    #通过抓取计算得到的路径，指定抓取箱子
    plan_path = gvm.get_variable("plan_path", per_reference=False, default=None)
    if not plan_path:
        self.logger.info("Plan path not found")
        return "find_pick_path"
    pick_item_id = plan_path[0]
    outputs["pick_box_id"] = pick_item_id
    
    items = list(filter(lambda x: x.additional_info.values[-3] == pick_item_id, items))
    if not items:
        raise "没有找到箱子,奇怪的bug"
    
    items_dict = {get_id(item.name):item for item in items}

    tips = tip_map.values()
    boxes = list(map(get_pallet_box, items))


    place_ws_id = items[0].additional_info.values[-1]
    place_workspace = planning_env.get_workspace_ros(str(place_ws_id))
    outputs["place_ws_id"] = place_ws_id
    if not place_workspace:
        raise Exception("Place workspace id not found")
    place_workspace_vertices = utils.get_box_vertices(
        place_workspace.get_bottom_pose().xyz_quat + place_workspace.get_dimensions())



    grasp_plans = []
    object_poses = []

    square_threshold = self.smart_data["square_threshold"]
    assert square_threshold >= 0 and square_threshold < 0.05, "square_threshold应该大于等于0且小于0.05m"
    is_square = (abs(items[0].primitives[0].dimensions[0] - items[0].primitives[0].dimensions[1]) < square_threshold)
    barcode_direction = 0
    if inputs["sku_info"]:
        barcode_direction = inputs["sku_info"]["barcode_direction"]
    
    #通过放置位置更新条码朝向    
    if place_ws_id == "6":
        barcode_direction = 4      
    else:
        barcode_direction = 0
        
    if barcode_direction == 0:
        self.logger.warning("无条码")
        slot_symmetry_angle = 180
        if is_square:
            slot_symmetry_angle = 90
    elif barcode_direction == 1 or barcode_direction == 3:
        self.logger.warning("对面有条码码")
        slot_symmetry_angle = 180
        is_square = False
    else:
        self.logger.warning("一面有条码")
        slot_symmetry_angle = 360
        is_square = False   
    slot_angles = list(range(0, 360, slot_symmetry_angle))
    # always consider 180 symmetry angle for tf_tip_obj first
    grasp_symmetry_angle = self.smart_data["grasp_symmetry_angle"]
    if grasp_symmetry_angle not in [45, 90, 180]:
        raise Exception("Pick an angle from [45, 90, 180].")
    grasp_angles = list(range(0, 360, grasp_symmetry_angle))

    ## get place slots
    slots = planning_env.get_unfinished_planned_items(place_ws_id)
    if not slots:
        ## TODO sync environment
        if self.smart_data["clear_motion_payload"]:
            gvm.set_variable("motion_payload", None, per_reference=True)
        if planning_env.get_planned_items(place_ws_id):
            self.logger.warning("放置位工作空间已满，没有可放置的位置，请清空放置位工作空间环境、连接放置生成模块")
            return "place_workspace_full"
        self.logger.warning("放置位工作空间没有生成放置方案，请确保place_plan_empty接口连接放置生成模块")
        return "place_plan_empty"


    ## only keep the bottom place slots
    bottom = min(slots, key = lambda item: item.origin.z)

    def in_bottom_layer(item):
        T = np.linalg.inv(place_workspace.get_bottom_pose().homogeneous)
        z1 = np.matmul(T, np.array(pose_to_list(bottom.origin)[:3] + [1.0]).transpose())[2]
        z2 = np.matmul(T, np.array(pose_to_list(item.origin)[:3] + [1.0]).transpose())[2]
        return (z2 - z1) < item.primitives[0].dimensions[2] / 2

    slots = list(filter(in_bottom_layer, slots))

    ## sort place slots
    if self.smart_data["sort_slots_by_human_label"]:
        slots = sorted(slots, key = lambda i: int((i.name).split("-")[1]))
    elif self.smart_data["sort_slots_by_corner_id"]:
        slot_boxes = list(map(get_pallet_box, slots))
        sorted_ids, _ = utils.sort_box(slot_boxes, place_workspace_vertices[place_corner_id])
        slots_np = np.array(slots)
        sorted_slots_np = slots_np[sorted_ids]
        slots = list(sorted_slots_np)
    else:
        # do nothing
        pass
    self.logger.info("sorted slot names:")
    for slot in slots:
        self.logger.info(slot.name)

        



    trans = SE3([0, 0, 0, 0, 0, 0, 1])

    for target in pick_workspace_vertices[pick_corner_heuristic:4] + pick_workspace_vertices[:pick_corner_heuristic]:
        ## sort all boxes by the target corner
        _, all_sorted_boxes = utils.sort_box(boxes, target)
        if not all_sorted_boxes:
            raise Exception("Boxes are all filtered out by sort_box! Boxes: {}".format(
                list(map(lambda b: b.to_vector(), boxes))))

        grasp_box = all_sorted_boxes[0]
        for _, tip in enumerate(tips):
            gp = GraspPlan()
            gp.tip = tip
            gp.objects.append(items_dict[grasp_box.id])
            gp.to_workspace_id = place_ws_id
            gp.from_workspace_id = pick_ws_id
            gp.reference_object_name = gp.objects[0].name
            gp.planned_items_ids.extend([slots[0].name])
            object_symmetry_poses = []
            if self.smart_data["fixed_grasp_pose"]["enable"]:
                fixed_endflange_poses = self.smart_data["fixed_grasp_pose"]["fixed_endflange_poses"]
                if self.smart_data["fixed_grasp_pose"]["modify_grasp_pose_by_size"]["enable"] and pick_workspace.get_type_string() == u'conveyor':
                    conveyor_box_alignment = self.smart_data["fixed_grasp_pose"]["modify_grasp_pose_by_size"]["conveyor_box_alignment"]
                    object_length_direction = self.smart_data["fixed_grasp_pose"]["modify_grasp_pose_by_size"]["object_length_direction"]
                    length_along_conveyor = 1 if object_length_direction == "y" else 0
                    modify_axis = self.smart_data["fixed_grasp_pose"]["modify_grasp_pose_by_size"]["modify_axis"]
                    refer_sku_dim = self.smart_data["fixed_grasp_pose"]["modify_grasp_pose_by_size"]["refer_sku_dimensions"]
                    cur_sku_dim = [inputs["sku_info"]["length"], inputs["sku_info"]["width"], inputs["sku_info"]["height"]]
                    sku_dim_diff_list = list(map(lambda d: d[0]-d[1], zip(cur_sku_dim, refer_sku_dim))) 
                    alignment_helper_dict = {"lower_left": [1, 1, 1], "upper_left": [1, -1, 1], "upper_right": [-1,-1, 1], "lower_right": [-1, 1, 1]}
                    alignment_helper = alignment_helper_dict[conveyor_box_alignment] # grasp_pose is related to alignment
                    dim_diff_helper = [0,1,2] if length_along_conveyor else [1,0,2] # grasp_pose is related to box_direction
                    modified_grasp_pose = []
                    for tf_robot_endflange in fixed_endflange_poses:
                        tf_robot_endflange = SE3(tf_robot_endflange)
                        # convert to conveyor coordinate from robot coordinate
                        tf_conveyor_endflange_list = ((pick_workspace.get_bottom_pose()).inv() * tf_map_robot * tf_robot_endflange).xyz_quat
                        # modeify fixed_grasp_pose by sku size(based conveyor coordinate)
                        tf_conveyor_endflange_list[0] = tf_conveyor_endflange_list[0] + modify_axis[0] * sku_dim_diff_list[dim_diff_helper[0]]/2 * alignment_helper[0]
                        tf_conveyor_endflange_list[1] = tf_conveyor_endflange_list[1] + modify_axis[1] * sku_dim_diff_list[dim_diff_helper[1]]/2 * alignment_helper[1]
                        tf_conveyor_endflange_list[2] = tf_conveyor_endflange_list[2] + modify_axis[2] * sku_dim_diff_list[dim_diff_helper[2]]   * alignment_helper[2]
                        # convert to robot coordinate from conveyor coordinate
                        tf_robot_endflange = (tf_map_robot.inv() * pick_workspace.get_bottom_pose() * SE3(tf_conveyor_endflange_list)).xyz_quat
                        modified_grasp_pose.append(tf_robot_endflange)
                    self.logger.warning(f"修正后的抓取pose为{modified_grasp_pose}")
                    fixed_endflange_poses = modified_grasp_pose
                tf_endflange_tip = planning_robot_ros.get_active_tool().get_tf_endflange_tip(tip.name)
                tf_map_box = SE3(grasp_box.get_pose_vector())
                for tf_robot_endflange in fixed_endflange_poses:
                    tf_robot_endflange = SE3(tf_robot_endflange)
                    tf_tip_obj = (tf_map_robot * tf_robot_endflange * tf_endflange_tip).inv() * tf_map_box
                    pose = Pose(*(tf_tip_obj.xyz_quat))
                    gp.object_tip_transforms.append(pose)
            else:
                ## construct two 180-symmetric tf_tip_object, note that this should be the same as that in box grasp plans
                for angle in grasp_angles:
                    trans_rotated = trans * SE3([0, 0, 0, 0, 0, math.sin(np.deg2rad(angle)/2.0), math.cos(np.deg2rad(angle)/2.0)]) * SE3([0, 0, 0, 1, 0, 0, 0])
                    pose = Pose()
                    pose.x, pose.y, pose.z, pose.qx, pose.qy, pose.qz, pose.qw = trans_rotated.xyz_quat
                    gp.object_tip_transforms.append(pose)
                    if show_valid_grasp_plans:
                        try:
                            remaining_boxes = list(map(get_pallet_box, filter(lambda box: box not in gp.objects, items_dict.values())))
                            tip_box = copy.deepcopy(grasp_box)
                            tip_box.length, tip_box.width = (tip.bounding_box.dimensions[0], tip.bounding_box.dimensions[1])
                            tip_box.transform(SE3(trans_rotated).inv().xyz_quat)
                            debugshow([grasp_box], remaining_boxes, [tip_box], title = "Showing all grasp plans")
                        except _DebugFastforward:
                            show_valid_grasp_plans = False
            for angle in slot_angles:
                object_symmetry_pose = SE3(pose_to_list(slots[0].origin)) * SE3([0, 0, 0, 0, 0, math.sin(np.deg2rad(angle)/2.0), math.cos(np.deg2rad(angle)/2.0)])
                object_symmetry_poses.append(object_symmetry_pose.xyz_quat)

            grasp_plans.append(gp)
            object_poses.append(object_symmetry_poses)

    if not grasp_plans or not object_poses:
        raise Exception("All grasp plans filtered out by planar collision or suction force.")

    grasp_plans, object_poses = zip(*sorted(zip(grasp_plans, object_poses), key=lambda zipped: len(zipped[0].objects), reverse=True))

    # outputs of the grasp plan.
    object_plan_pairs = {"current_index" : 1,
                         "grasp_plan": grasp_plans,
                         "object_poses": object_poses}
    gvm.set_variable(
        "{}&{}".format(self.name, self.smart_data["reuse_tag"]),
        object_plan_pairs,
        per_reference=True)

    outputs["grasp_plan"] = grasp_plans[0]
    outputs["object_poses"] = object_poses[0]
    self.logger.info(f"grasp plan 已规划出指定箱子{plan_path[0][0]},更新路径")
    self.logger.info(f"抓取箱子ID为{plan_path[0][0]}")
    plan_path.pop(0)
    gvm.set_variable("plan_path", plan_path, per_reference=False)
    return "success"
