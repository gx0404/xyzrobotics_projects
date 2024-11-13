# -*- coding: utf-8 -*-
import time
from queue import Empty, Queue
from threading import Thread

from flask import Blueprint, current_app, request

from apps import _, make_json_response, mp, openapi, settings, validator
from apps.base_app.flask_hook import req_log
from apps.exceptions import XYZBaseError
from apps.log import wcs_log
from wcs_adaptor.depalletize.entity import DPTTask as Task
from wcs_adaptor.depalletize.schema import (
    GetMultiPalStartInfoSchema,
    IsWsReadyInputSchema,
    OrderCreateSchema,
    SingleTaskCreateSchema,
    TaskCreateSchema,
)
from wcs_adaptor.depalletize.wcs_request import report_plan_result
from wcs_adaptor.entity import LiteOrder as Order
from wcs_adaptor.entity import Pallet
from wcs_adaptor.enums import TaskStatus, TaskType
from wcs_adaptor.exceptions import (
    OrderDuplicateError,
    PalletNotFoundError,
    TaskDuplicateError,
    TaskNotFoundError,
    ValidationError,
    VisionError,
    XYZValidationError,
)
from wcs_adaptor.helpers import backup_manager_wrapper
from wcs_adaptor.manager import order_manager, task_manager, workspace_manager
from wcs_adaptor.settings import wcs_settings
from wcs_adaptor.models import PalletData
from apps.models import db
from wcs_adaptor.depalletize.rafcon import get_last_task_type

bp = Blueprint("wcs", __name__, url_prefix="/api/wcs")
  

# 任务类型定义，便于打印信息
TASK_TYPE_DESC = {
    0:"拣配任务",
    10:"合托任务",
    1:"笼车空箱回收任务",
    3:"输送线空箱回收任务",
}

# ---------
#  基础功能
# ---------

HEARTBEAT_TIME = Queue()


def heartbeat_check():
    from apps import cached_app

    app = cached_app()
    while True:
        try:
            HEARTBEAT_TIME.get(timeout=settings.HEARTBEAT_TIMEOUT)
        except Empty:
            # 心跳检测超时报警
            app.mp.wcs_connection_status.disconnected()
            wcs_log.info("wcs lost connection")
        else:
            # 未超时
            app.mp.wcs_connection_status.connected()
            wcs_log.info("wcs connected")


def start_heartbeat_check_thread():
    """开启心跳检测线程."""
    wcs_log.info("wcs heartbeat check thread start")
    t = Thread(target=heartbeat_check, name="WcsHeartbeatMonitor", daemon=True)
    t.start()


@bp.route("/is_system_ready", methods=["GET", "POST"])
# @req_log(log=wcs_log)       # 用作心跳检测的话不要输出日志了
@openapi.api_doc(tags=["WCS"])
def is_system_ready():
    """询问系统是否准备好.

    Returns:
        ---
        {
            "status": "ready",
            "message": "系统准备好了"
        }
        ---
    """
    # 系统运行状况
    output_data = {
        "status": 0 if current_app.status == "ready" else -1,
        "message": current_app.status,
    }
    HEARTBEAT_TIME.put(time.time())
    return output_data


"""回报机器人状态"""
@bp.route("/robot_status", methods=["GET", "POST"])
# @req_log(log=wcs_log)      
@openapi.api_doc(tags=["WCS"])
def robot_status():
    from xyz_motion import RobotDriver,create_kinesolver
    from py_xyz_robot import robot_config
    import numpy as np
    # time.sleep(0.05)
    try:
        # robot_name = robot_config.get_robot_name(0)
        # kinematic_solver = create_kinesolver(robot_name)
        # r = RobotDriver(0)
        # joints = kinematic_solver.convert_four_dof_to_six(list(r.get_joints()))     
        # joints.pop(2)
        # joints[3]+=1.5707963267948966
        # joints = np.rad2deg(joints).tolist()
        
        # from xyz_io_client.io_client import get_digit_input
        # joints = []
        # abs_aixs_list = [get_digit_input("3",i) for i in range(6,12)] 
        # negative_aixs_list = [get_digit_input("3",i) for i in range(12,18)] 
        # do_value_list = [get_digit_input("3",i) for i in range(109,141)]  
        # for index, value in enumerate(negative_aixs_list):
        #     if value == 1:
        #         joints.append(-abs_aixs_list[index]/100)
        #     else:
        #         joints.append(abs_aixs_list[index]/100)
        
        # do_value_list = [get_digit_input("3",i) for i in range(109,141)]   
        # do_value_dict = {}       
        # for index,value in enumerate(do_value_list):
        #     key = "DO"+str(index)
        #     do_value_dict[key] = do_value_list[bool(value)]

        from plc import plc_snap7
        plc = plc_snap7()
        plc.connect("192.168.36.5")
        
        joints = [i/100 for i in plc.get_ints("db",0,6,3)]
        do_value_dict = {}
        do_value_list = plc.get_ints("db",218,32,3)
        for index,value in enumerate(do_value_list):
            key = "DO"+str(index+1)
            do_value_dict[key] = bool(do_value_list[index])
            
        di_value_dict = {}
        di_value_list = plc.get_ints("db",282,32,3)
        for index,value in enumerate(di_value_list):
            key = "DI"+str(index+1)
            di_value_dict[key] = bool(di_value_list[index])   
                 
        robot_status = bool(plc.get_ints("db",346,1,3)[0])
        system_status =  current_app.status      
        plc.dis_connected()                      
        return make_json_response(error=0,error_msg="",robot_joints = joints,
                                  do_value_dict = do_value_dict,di_value_dict = di_value_dict,                                  
                                  robot_status="running"if robot_status else "stop",
                                  system_status="running" if system_status=="ready" else "stop")    
    except Exception as e:
        wcs_log.error(f"robot_status error,error is :{e}")
        #import ipdb;ipdb.set_trace()
        return make_json_response(error=1,error_msg=str(e),rrobot_joints = [],
                                  do_value_dict = {},di_value_dict={},
                                  robot_status="stop", system_status="stop")     



@bp.route("/agv_is_ready", methods=["POST"])
@req_log(log=wcs_log)
@backup_manager_wrapper()
@validator()
@openapi.api_doc(tags=["WCS", "工作空间"], body=IsWsReadyInputSchema)
def agv_is_ready(body: IsWsReadyInputSchema):
    # 获取当前空间对象, 不存在则新增.
    ws = workspace_manager.create_or_modify(ws_id=body.ws_id)
    # 清空工作空间.
    ws.clear()
    if body.pallet_id:
        pallet = ws.get_item(id=body.pallet_id)
        # 该托盘 ID 不存在, 则新增托盘
        if pallet is None:
            pallet = Pallet(pallet_id=body.pallet_id, ws_id=ws.ws_id)
            ws.add_item(pallet)
        pallet.clear()
    # 设置工作空间状态为已就绪
    ws.ready()
    mp.order.info(_("agv在工作空间({0})已到位").format(body.ws_id))
    return make_json_response(error=0)    
    
    
    
# @bp.route("/notice_pick_ws_ready", methods=["POST"])
# @req_log(log=wcs_log)
# @backup_manager_wrapper()
# @validator()
# @openapi.api_doc(tags=["WCS", "工作空间"], body=IsWsReadyInputSchema)
# def notice_pick_ws_ready(body: IsWsReadyInputSchema):
#     """通知夹取空间已就绪.

#     Raises:
#         ValidationError: 参数验证错误.
#         TaskNotFoundError: 未找到任务.

#     Returns:
#         ---
#         {
#             "code": 0,
#             "msg": "success",
#             "data": {}
#         }
#         ---
#     """
#     # 获取当前空间对象, 不存在则新增.
#     ws = workspace_manager.create_or_modify(ws_id=body.ws_id)
#     # 清空工作空间.
#     ws.clear()
#     if body.pallet_id:
#         pallet = ws.get_item(id=body.pallet_id)
#         # 该托盘 ID 不存在, 则新增托盘
#         if pallet is None:
#             pallet = Pallet(pallet_id=body.pallet_id, ws_id=ws.ws_id)
#             ws.add_item(pallet)
#         pallet.clear()
#     # 设置工作空间状态为已就绪
#     ws.ready()
#     mp.order.info(_("抓取位({0})已到位").format(body.ws_id))
#     return make_json_response(error=0)


# @bp.route("/notice_place_ws_ready", methods=["POST"])
# @req_log(log=wcs_log)
# @backup_manager_wrapper()
# @validator()
# @openapi.api_doc(tags=["WCS", "工作空间"], body=IsWsReadyInputSchema)
# def notice_place_ws_ready(body: IsWsReadyInputSchema):
#     """放置工作空间已就绪.

#     Raises:
#         ValidationError: 参数验证错误.
#         TaskNotFoundError: 未找到任务.

#     Returns:
#         ---
#         {
#             "code": 0,
#             "msg": "success",
#             "data": {}
#         }
#         ---
#     """
#     # 获取当前空间对象, 不存在则新增.
#     ws = workspace_manager.create_or_modify(ws_id=body.ws_id)
#     # 清空工作空间.
#     ws.clear()
#     if body.pallet_id:
#         pallet = ws.get_item(id=body.pallet_id)
#         # 该托盘 ID 不存在, 则新增托盘
#         if pallet is None:
#             pallet = Pallet(pallet_id=body.pallet_id, ws_id=ws.ws_id)
#             ws.add_item(pallet)
#         pallet.clear()
#     # 设置工作空间状态为已就绪
#     ws.ready()
#     mp.order.info(_("放置位({0})已到位").format(ws.ws_id))
#     return make_json_response(error=0)




@bp.route("/terminate_task", methods=["GET", "POST"])
@req_log(log=wcs_log)
@backup_manager_wrapper()
@openapi.api_doc(tags=["WCS", "任务管理"])
def terminate_task():
    """WCS请求提前结束任务.

    Query:
        pass

    Body:
        ---
        {
            "task_id": "task_1",
        }
        ---

    Returns:
        ---
        {
            "code": 0,
            "msg": "success",
            "data": {}
        }
        ---

    Raises:
        ValidationError: 参数输入异常.
        TaskNotFoundError: 任务不存在异常.

    """
    data = request.args if request.method == "GET" else request.get_json()
    if "task_id" not in data:
        raise ValidationError(error_message="Input format is wrong: no task id")

    task_id = data["task_id"]
    task = task_manager.get_task_by_id(task_id)

    if not task:
        raise TaskNotFoundError(task_id=task_id)
    #import ipdb;ipdb.set_trace()
    task.terminate(auto_remove=False)
    mp.order.error("wcs下发终止任务")
    #task_manager.remove(task)
    return make_json_response(error=0)





# ---------
#  拣配任务
# ---------
@bp.route("/depal_task", methods=["POST"])
@req_log(log=wcs_log)
@backup_manager_wrapper()
@validator()
@openapi.api_doc(tags=["WCS", "创建任务"], request_body=SingleTaskCreateSchema)
def depal_task(body: SingleTaskCreateSchema):
    """创建拣配任务.

    Returns:
        ---
        {
            "code": 0,
            "msg": "success",
            "data": {}
        }
        ---

    Raises:
        ValidationError: 接口参数输入异常.
        TaskDuplicateError: 传入了重复的任务.

    """

    if body.sku_type==2:
        sku_info = {
                "sku_id": "a",
                "length": 405,
                "width": 303,
                "height": 160,
                "weight": 5,
                "sku_num": -1
                }   
    elif body.sku_type==0:
        sku_info = {
                "sku_id": "b",
                "length": 400,
                "width": 300,
                "height": 230,
                "weight": 5,
                "sku_num": -1
                }              
    elif body.sku_type==1:
        sku_info = {
                "sku_id": "c",
                "length": 600,
                "width": 400,
                "height": 230,
                "weight": 5,
                "sku_num": -1
                } 
    else:
        mp.order.error(f"wcs下发拣配任务sku类型错误")        
        return make_json_response(error=1,error_message="sku_type error")
    
    #判断上一次任务类型
    last_data = get_last_task_type()
    last_task = last_data["0"]
    if last_task['task_type'] != TaskType.SINGLE_DEPAL.value:
        mp.order.info(f"上次任务类型为{TASK_TYPE_DESC[last_task['task_type'].value]},需要清空所有环境")  
        pallet_clear_list = ["0","1","2","3","4","5","6","7","8"]
    else:
        pallet_clear_list = body.pallet_clear_list      
    
    
    try:
        from xyz_env_manager.client import get_planning_environment
        from xyz_motion import PlanningEnvironmentRos
        pl = get_planning_environment()
        planning_env = PlanningEnvironmentRos.from_ros_msg(pl)
        container_items_2 = planning_env.get_container_items("2")
        container_items_3 = planning_env.get_container_items("3")
        if container_items_2:
            sku_dimension_2 = container_items_2[0].primitives[0].dimensions
            sku_dimension_2 = list(map(lambda x:round(x,2),sku_dimension_2))
        if container_items_3:
            sku_dimension_3 = container_items_3[0].primitives[0].dimensions
            sku_dimension_3 = list(map(lambda x:round(x,2),sku_dimension_3))                
    except ImportError as err:
        return make_json_response(error=5,error_message="环境节点未启动,请在HMI中启动环境节点") 
    
    #检查wcs下发的托盘条码是否和目标条码一致
    for key,item in body.pick_tote_data.items():
        if key not in body.pallet_tote_data.keys():
            mp.order.error(f"wcs下发拣配任务目标箱位置号不在托盘位置号里")        
            return make_json_response(error=1,error_message="目标箱位置号不在托盘位置号")   
        if item["barcode"] != body.pallet_tote_data[key]["barcode"]:
            mp.order.error(f"wcs下发拣配任务目标箱条码和托盘对应条码不一致")
            return make_json_response(error=2,error_message="目标箱条码和托盘对应条码不一致")
        if body.sku_type==0:
            if item["to_ws"]=="2":
                body.lower_speed = True
                if container_items_2:
                    if sku_dimension_2!=[0.4,0.3,0.23] and ("2" not in pallet_clear_list):
                        mp.order.error(f"当前空间2环境已存在大欧箱,订单却下发空间2中欧箱")  
                        return make_json_response(error=6,error_message="当前空间2环境已存在大欧箱,订单却下发空间2中欧箱")   
            elif item["to_ws"]=="3":
                body.lower_speed = True
                if container_items_3:
                    if sku_dimension_3!=[0.4,0.3,0.23] and ("3" not in pallet_clear_list):
                        mp.order.error(f"当前空间3环境已存在大欧箱,订单却下发空间2中欧箱")  
                        return make_json_response(error=6,error_message="当前空间3环境已存在大欧箱,订单却下发空间2中欧箱")  
            elif item["to_ws"]=="6":
                pass
            else:
                mp.order.error(f"中欧箱目标箱终点必须为笼车或者输送线")     
                return make_json_response(error=3,error_message="中欧箱目标箱终点必须为笼车或者输送线")          
                                       
        if body.sku_type==1:
            if item["to_ws"]=="2":
                if container_items_2:
                    if sku_dimension_2!=[0.6,0.4,0.23] and ("2" not in pallet_clear_list):
                        mp.order.error(f"当前空间2环境已存在中欧箱,订单却下发空间2大欧箱")  
                        return make_json_response(error=6,error_message="当前空间2环境已存在中欧箱,订单却下发空间2大欧箱")   
            elif item["to_ws"]=="3":
                if container_items_3:
                    if sku_dimension_3!=[0.6,0.4,0.23]and ("3" not in pallet_clear_list):
                        mp.order.error(f"当前空间3环境已存在中欧箱,订单却下发空间2大欧箱")  
                        return make_json_response(error=6,error_message="当前空间3环境已存在中欧箱,订单却下发空间2大欧箱")  
            elif item["to_ws"]=="6":
                pass
            else:
                mp.order.error(f"大欧箱目标箱终点必须为笼车或者输送线")     
                return make_json_response(error=3,error_message="大欧箱目标箱终点必须为笼车或者输送线")               
                              
                  
    # 创建新任务
    task = Task(
        task_id=body.task_id,
        task_type=TaskType.SINGLE_DEPAL,
        target_num=-1,
        sku_info=sku_info,
        from_ws=body.from_ws,
        to_ws=body.to_ws,
        pallet_tote_data=body.pallet_tote_data,
        pick_tote_data=body.pick_tote_data,
        pallet_clear_list = pallet_clear_list,
        customized_data = body.pick_tote_data,
        lower_layer = body.lower_layer,
        lower_speed = body.lower_speed,
    )
    task_manager.append(task)

    # NOTICE: 单拆默认清空工作空间, 单拆单码通常以拆(码)一个托盘为单位作为WCS交互的一个任务，
    #  不需要其他信号，在下一个任务中默认以新托盘开始，如实际情况不同，请按需修改
    workspace_manager.create_or_modify(ws_id="0", is_ready=True)
    workspace_manager.create_or_modify(ws_id=task.to_ws, is_ready=False)
    
    #通过wcs下发的清空托盘列表更新托盘就位
    for ws_id in task.pallet_clear_list:
        workspace_manager.create_or_modify(ws_id=ws_id, is_ready=True)

    message = _("已收到拣配任务({0})，任务数量: {1}").format(task.task_id, task.target_num)
    mp.order.info(message)
    model = PalletData.query.first()
    if not model:
        model = PalletData(current_direction="",pallet_tote_data={},pick_tote_data={},\
            cache_pallet_tote_data={},path={},pallet_tote_data_2={},pallet_tote_data_3={}\
            ,pallet_tote_data_7={},pallet_tote_data_8={})
        db.session.add(model)
        db.session.commit()
    else:    
        model.current_direction = ""
        model.pallet_tote_data = body.pallet_tote_data
        model.pick_tote_data = body.pick_tote_data
        model.cache_pallet_tote_data = {}
        model.path = {}
        model.pallet_tote_data_2 = {}
        model.pallet_tote_data_3 = {}
        model.pallet_tote_data_7 = {}
        model.pallet_tote_data_8 = {}
        db.session.add(model)
        db.session.commit()
        

    return make_json_response(error=0)



# ---------
#  合托任务
# ---------
@bp.route("/merge_task", methods=["POST"])
@req_log(log=wcs_log)
@backup_manager_wrapper()
@validator()
@openapi.api_doc(tags=["WCS", "创建任务"], request_body=SingleTaskCreateSchema)
def merge_task(body: SingleTaskCreateSchema):
    """创建合托任务.

    Returns:
        ---
        {
            "code": 0,
            "msg": "success",
            "data": {}
        }
        ---

    Raises:
        ValidationError: 接口参数输入异常.
        TaskDuplicateError: 传入了重复的任务.

    """

    if body.sku_type==2:
        sku_info = {
                "sku_id": "a",
                "length": 405,
                "width": 303,
                "height": 160,
                "weight": 5,
                "sku_num": -1
                }   
    elif body.sku_type==0:
        sku_info = {
                "sku_id": "b",
                "length": 400,
                "width": 300,
                "height": 230,
                "weight": 5,
                "sku_num": -1
                }              
    elif body.sku_type==1:
        sku_info = {
                "sku_id": "c",
                "length": 600,
                "width": 400,
                "height": 230,
                "weight": 5,
                "sku_num": -1
                } 
    else:
        return make_json_response(error=0,error_message="sku_type error")        

    #判断上一次任务类型
    last_data = get_last_task_type()
    last_task = last_data["0"]
 
    if last_task['task_type'] != TaskType.MERGE_TASK.value:
        mp.order.info(f"上次任务类型为{TASK_TYPE_DESC[last_task['task_type'].value]},需要清空所有环境")  
        pallet_clear_list = ["0","1","2","3","4","5","6","7","8"]
    else:
        pallet_clear_list = body.pallet_clear_list       
                
    # 创建新任务
    task = Task(
        task_id=body.task_id,
        task_type=TaskType.MERGE_TASK,
        target_num=-1,
        sku_info=sku_info,
        from_ws=body.from_ws,
        to_ws=body.to_ws,
        from_pallet_tote_data=body.from_pallet_tote_data,
        to_pallet_tote_data=body.to_pallet_tote_data,
        pallet_clear_list = pallet_clear_list,
    )
    task_manager.append(task)

    # NOTICE: 单拆默认清空工作空间, 单拆单码通常以拆(码)一个托盘为单位作为WCS交互的一个任务，
    #  不需要其他信号，在下一个任务中默认以新托盘开始，如实际情况不同，请按需修改
    workspace_manager.create_or_modify(ws_id=task.from_ws, is_ready=False)
    workspace_manager.create_or_modify(ws_id=task.to_ws, is_ready=False)

    message = _("已收到合托任务({0})，任务数量: {1}").format(task.task_id, task.target_num)
    mp.order.info(message)
    model = PalletData.query.first()
    if not model:
        model = PalletData(current_direction="",pallet_tote_data={},pick_tote_data={},\
            cache_pallet_tote_data={},path={},pallet_tote_data_2={},pallet_tote_data_3={}\
            ,pallet_tote_data_7={},pallet_tote_data_8={})
        db.session.add(model)
        db.session.commit()
    else:    
        model.current_direction = ""
        model.pallet_tote_data = body.to_pallet_tote_data
        model.pick_tote_data = {}
        model.cache_pallet_tote_data = body.from_pallet_tote_data
        model.path = {}
        model.pallet_tote_data_2 = {}
        model.pallet_tote_data_3 = {}
        model.pallet_tote_data_7 = {}
        model.pallet_tote_data_8 = {}
        db.session.add(model)
        db.session.commit()
        
    return make_json_response(error=0)


# ---------
#  输送线混码接口
# ---------
@bp.route("/conveyor_pal_task", methods=["POST"])
@req_log(log=wcs_log)
@backup_manager_wrapper()
@validator()
@openapi.api_doc(tags=["WCS", "创建任务"], request_body=SingleTaskCreateSchema)
def conveyor_pal_task(body: SingleTaskCreateSchema):
    """创建任务.

    Returns:
        ---
        {
            "code": 0,
            "msg": "success",
            "data": {}
        }
        ---

    Raises:
        ValidationError: 接口参数输入异常.
        TaskDuplicateError: 传入了重复的任务.

    """
    sku_info = {
        "sku_id": "null",
        "length": 600,
        "width": 400,
        "height": 230,
        "weight": 5,
        "sku_num": -1
        } 

    #判断上一次任务类型
    last_data = get_last_task_type()
    last_task = last_data["0"]
    
    if last_task['task_type'] != TaskType.MULTI_PAL_ONLINE.value:
        mp.order.info(f"上次任务类型为{TASK_TYPE_DESC[last_task['task_type'].value]},需要清空所有环境")  
        pallet_clear_list = ["0","1","2","3","4","5","6","7","8"]
    else:
        pallet_clear_list = body.pallet_clear_list    
            
    # 创建新任务
    task = Task(
        task_id=body.task_id,
        task_type=TaskType.MULTI_PAL_ONLINE,
        target_num=-1,
        sku_info=sku_info,
        from_ws=body.from_ws,
        to_ws=body.to_ws,
        pallet_clear_list = pallet_clear_list  
    )
    task_manager.append(task)

    # NOTICE: 输送线混码任务默认需要等待所有托盘到位
    #  不需要其他信号，在下一个任务中默认以新托盘开始，如实际情况不同，请按需修改
    wait_place_list = ["0", "1", "2", "3", "4", "5", "6"]
    for i in wait_place_list:        
        workspace_manager.create_or_modify(ws_id=i, is_ready=True)
    
    
    message = _("已收到输送线混码任务({0})，任务数量: {1}").format(task.task_id, -1)
    mp.order.info(message)
    
    model = PalletData.query.first()
    if not model:
        model = PalletData(current_direction="",pallet_tote_data={},pick_tote_data={},\
            cache_pallet_tote_data={},path={},pallet_tote_data_2={},pallet_tote_data_3={}\
            ,pallet_tote_data_7={},pallet_tote_data_8={})
        db.session.add(model)
        db.session.commit()
    else:    
        model.current_direction = ""
        model.pallet_tote_data = body.to_pallet_tote_data
        model.pick_tote_data = {}
        model.cache_pallet_tote_data = body.from_pallet_tote_data
        model.path = {}
        model.pallet_tote_data_2 = {}
        model.pallet_tote_data_3 = {}
        model.pallet_tote_data_7 = {}
        model.pallet_tote_data_8 = {}
        db.session.add(model)
        db.session.commit()   
             
    return make_json_response(error=0)

@bp.route("/conveyor_pal_task/finish", methods=["GET", "POST"])
@req_log(log=wcs_log)
@backup_manager_wrapper()
@openapi.api_doc(tags=["WCS", "任务管理"])
def conveyor_pal_task_finish():
    """WCS请求提前结束任务.

    Query:
        pass

    Body:
        ---
        {
            "task_id": "task_1",
        }
        ---

    Returns:
        ---
        {
            "code": 0,
            "msg": "success",
            "data": {}
        }
        ---

    Raises:
        ValidationError: 参数输入异常.
        TaskNotFoundError: 任务不存在异常.

    """
    data = request.args if request.method == "GET" else request.get_json()
    if "task_id" not in data:
        raise ValidationError(error_message="Input format is wrong: no task id")

    task_id = data["task_id"]
    task = task_manager.get_task_by_id(task_id)

    if not task:
        raise TaskNotFoundError(task_id=task_id)
    if task.task_type!=TaskType.MULTI_PAL_ONLINE:
        return make_json_response(error=99,error_message="任务类型错误")
    #import ipdb;ipdb.set_trace()
    task.terminate(auto_remove=False)
    mp.order.error("wcs下发输送线空箱回收任务结束")
    #task_manager.remove(task)
    return make_json_response(error=0)


# ---------
#  笼车单码接口
# ---------
@bp.route("/pallet_pal_task", methods=["POST"])
@req_log(log=wcs_log)
@backup_manager_wrapper()
@validator()
@openapi.api_doc(tags=["WCS", "创建任务"], request_body=SingleTaskCreateSchema)
def pallet_pal_task(body: SingleTaskCreateSchema):
    """创建任务.

    Returns:
        ---
        {
            "code": 0,
            "msg": "success",
            "data": {}
        }
        ---

    Raises:
        ValidationError: 接口参数输入异常.
        TaskDuplicateError: 传入了重复的任务.

    """
    sku_info = {
        "sku_id": "null",
        "length": 600,
        "width": 400,
        "height": 230,
        "weight": 5,
        "sku_num": -1
        } 
    if body.from_ws not in ["2","3"]:
        mp.order.error(f"笼车空箱回收任务来料位只支持2,3,但实际为{body.from_ws}")
        return make_json_response(error=99,error_message=f"笼车空箱回收任务来料位只支持2,3,但实际为{body.from_ws}")

    #判断上一次任务类型
    last_data = get_last_task_type()
    last_task = last_data["0"]
        
    if last_task['task_type'] != TaskType.SINGLE_PAL.value:
        mp.order.info(f"上次任务类型为{TASK_TYPE_DESC[last_task['task_type'].value]},需要清空所有环境")  
        pallet_clear_list = ["0","1","2","3","4","5","6","7","8"]
    else:
        pallet_clear_list = body.pallet_clear_list 
            
    # 创建新任务
    task = Task(
        task_id=body.task_id,
        task_type=TaskType.SINGLE_PAL,
        target_num=-1,
        sku_info=sku_info,
        from_ws=body.from_ws,
        to_ws=body.to_ws,
        pallet_clear_list = pallet_clear_list,
        customized_data = body.customized_data
    )
    task_manager.append(task)

    # NOTICE: 输送线混码任务默认需要等待所有托盘到位
    #  不需要其他信号，在下一个任务中默认以新托盘开始，如实际情况不同，请按需修改
    wait_place_list = ["0", "1", "2", "3", "4", "5", "6"]
    for i in wait_place_list:        
        workspace_manager.create_or_modify(ws_id=i, is_ready=True)

    message = _("已收到笼车单码任务({0})，任务数量: {1}").format(task.task_id, -1)
    mp.order.info(message)
        
    return make_json_response(error=0)

# ---------
#  单拆接口
# ---------
@bp.route("/single_class_depal_task", methods=["POST"])
@req_log(log=wcs_log)
@backup_manager_wrapper()
@validator()
@openapi.api_doc(tags=["WCS", "创建任务"], request_body=SingleTaskCreateSchema)
def single_class_depal_task(body: SingleTaskCreateSchema):
    """创建单拆任务.

    Returns:
        ---
        {
            "code": 0,
            "msg": "success",
            "data": {}
        }
        ---

    Raises:
        ValidationError: 接口参数输入异常.
        TaskDuplicateError: 传入了重复的任务.

    """
    if body.target_num is not None:
        target_num = body.target_num
    elif body.sku_info.sku_num is not None:
        target_num = body.sku_info.sku_num
    else:
        raise ValidationError(_("必须指定 target_num 或 sku_info.sku_num"))

    # 创建新任务
    task = Task(
        task_id=body.task_id,
        task_type=TaskType.SINGLE_DEPAL,
        target_num=target_num,
        sku_info=body.sku_info,
        from_ws=body.from_ws,
        to_ws=body.to_ws,
    )
    task_manager.append(task)

    # NOTICE: 单拆默认清空工作空间, 单拆单码通常以拆(码)一个托盘为单位作为WCS交互的一个任务，
    #  不需要其他信号，在下一个任务中默认以新托盘开始，如实际情况不同，请按需修改
    workspace_manager.create_or_modify(ws_id=task.from_ws, is_ready=False)
    workspace_manager.create_or_modify(ws_id=task.to_ws, is_ready=True)

    message = _("已收到单拆任务({0})，任务数量: {1}").format(task.task_id, task.target_num)
    mp.order.info(message)
    return make_json_response(error=0)


# ---------
#  单码接口
# ---------
@bp.route("/single_class_pal_task", methods=["POST"])
@req_log(log=wcs_log)
@backup_manager_wrapper()
@validator()
@openapi.api_doc(tags=["WCS", "创建任务"], request_body=SingleTaskCreateSchema)
def single_class_pal_task(body: SingleTaskCreateSchema):
    """创建单码任务.

    Raises:
        ValidationError: 接口参数输入异常.
        TaskDuplicateError: 传入了重复的任务.

    Returns:
        ---
        {
            "code": 0,
            "msg": "success",
            "data": {}
        }
        ---

    """
    # if body.target_num is not None:
    #     target_num = body.target_num
    # elif body.sku_info.sku_num is not None:
    #     target_num = body.sku_info.sku_num
    # else:
    #     raise ValidationError(_("必须指定 target_num 或 sku_info.sku_num"))
    
    target_num = body.target_num    
    
    task = Task(
        task_id=body.task_id,
        task_type=TaskType.SINGLE_PAL,
        target_num=target_num,
        sku_info=body.sku_info,
        from_ws=body.from_ws,
        to_ws=body.to_ws,
    )
    task_manager.append(task)

    # NOTICE: 单码默认清空工作空间, 单码通常以拆(码)一个托盘为单位作为WCS交互的一个任务，
    #  不需要其他信号，在下一个任务中默认以新托盘开始，如实际情况不同，请按需修改
    workspace_manager.create_or_modify(ws_id=task.from_ws, is_ready=True)
    workspace_manager.create_or_modify(ws_id=task.to_ws, is_ready=True)

    message = _("已收到单码任务({0})，任务数量: {1}").format(task.task_id, task.target_num)
    mp.order.info(message)
    return make_json_response(error=0)


# ---------
#  混拆接口
# ---------
@bp.route("/multi_class_depal_task", methods=["POST"])
@req_log(log=wcs_log)
@backup_manager_wrapper()
@validator()
@openapi.api_doc(tags=["WCS", "创建任务"], request_body=TaskCreateSchema)
def multi_class_depal_task(body: TaskCreateSchema):
    """创建一个混拆任务.

    Returns:
        ---
        {
            "code": 0,
            "msg": "success",
            "data": {}
        }
        ---

    Raises:
        ValidationError: 接口参数输入异常.
        TaskDuplicateError: 传入了重复的任务.
    """
    # 混拆一般无sku信息, target_num默认为-1
    task = Task(
        task_id=body.task_id,
        task_type=TaskType.MULTI_DEPAL,
        target_num=body.target_num or -1,
        from_ws=body.from_ws,
        to_ws=body.to_ws,
    )
    task_manager.append(task)

    # NOTICE: 混拆是否清空工作空间，依项目而定
    workspace_manager.create_or_modify(ws_id=task.from_ws, is_ready=False)
    workspace_manager.create_or_modify(ws_id=task.to_ws, is_ready=False)

    message = _("已收到混拆任务({0})，任务数量: {1}").format(task.task_id, task.target_num)
    mp.order.info(message)
    return make_json_response(error=0)


# --------------
#  混码online接口
# --------------
@bp.route("/multi_class_pal_task_online", methods=["POST"])
@req_log(log=wcs_log)
@backup_manager_wrapper()
@validator()
@openapi.api_doc(tags=["WCS", "创建任务"], request_body=SingleTaskCreateSchema)
def multi_class_pal_task_online(body: SingleTaskCreateSchema):
    """创建混码任务.

    Raises:
        ValidationError: 接口参数输入异常.
        TaskDuplicateError: 传入了重复的任务.

    Returns:
        ---
        {
            "code": 0,
            "msg": "success",
            "data": {}
        }
        ---

    """
    if body.target_num is not None:
        target_num = body.target_num
    elif body.sku_info.sku_num is not None:
        target_num = body.sku_info.sku_num
    else:
        raise ValidationError(_("必须指定 target_num 或 sku_info.sku_num"))

    task = Task(
        task_id=body.task_id,
        task_type=TaskType.MULTI_PAL_ONLINE,
        target_num=target_num,
        sku_info=body.sku_info,
        from_ws=body.from_ws,
        to_ws=body.to_ws,
    )
    task_manager.append(task)

    # NOTICE: 混拆是否清空工作空间，依项目而定
    workspace_manager.create_or_modify(ws_id=task.from_ws, is_ready=False)
    workspace_manager.create_or_modify(ws_id=task.to_ws, is_ready=False)
    message = _("已收到混码任务({0})，任务数量: {1}").format(task.task_id, task.target_num)
    mp.order.info(message)
    return make_json_response(error=0)


# ----------------
#  混码offline接口
# ----------------
@bp.route("/multi_class_pal_task/", methods=["POST"])
@req_log(log=wcs_log)
@backup_manager_wrapper()
@validator()
@openapi.api_doc(tags=["WCS", "离线混码", "创建任务"], request_body=OrderCreateSchema)
def multi_class_pal_task_offline(body: OrderCreateSchema):
    """创建混码offline任务.

    Raises:
        ValidationError: 接口参数输入异常.
        TaskDuplicateError: 传入了重复的任务.
    """
    # 离线混码规划不允许最大重量超过10千克
    for sku in body.sku_info:
        if sku.weight > 10:
            raise ValidationError(
                f"离线混码物料重量最大不允许超过10，SKU({sku.sku_id})的重量为{sku.weight}"
            )

    from placing_msgs import (
        BoxInfo_pb2,
        MixPalletizeParam_pb2,
        Pose_pb2,
        SearchParam_pb2,
        ToolInfo_pb2,
    )

    from apps.ext.offline_mix_planner.entity import PlanningResult
    from apps.ext.offline_mix_planner.planner import OfflineMixedTaskPlanner

    order_id = body.order_id
    if order := order_manager.get_order_by_id(order_id):
        raise OrderDuplicateError(order_id)

    ################################用户设置#############################
    box_type = "box"  # "box" | "tote"
    sku_tool_map = {}  # eg. sku_tool_map = {"A":0,"B":1}  "sku_id":tool_id

    # mixed palletize placing parameters
    mpp = MixPalletizeParam_pb2.MixPalletizeParam()
    # mxied palletize searching parameters
    sp = SearchParam_pb2.SearchParam()

    # 注意： 确保Tool Tip的尺寸以及Tool上其他障碍物的大小与环境中的一致
    tool_1 = ToolInfo_pb2.ToolInfo()  # 工具信息
    tool_1.tip_size.length = 0.6
    tool_1.tip_size.width = 0.4
    tool_1.suction_force = 150
    # 如果Tool由其他障碍物组成，需要在tool_info中设置addon_size和tf_tip_addon，
    # 具体参数设置可以参考 https://xyz-robotics.atlassian.net/l/cp/C0GNG3pK
    # tool_info.addon_size.extend(...) # 工具附加物体长宽，单位m
    # tool_info.tf_tip_addon.extend(...) # 工具附加物体相对tip的变换, tip frame z向下, 单位m
    mpp.tool_info.extend([tool_1])

    mpp.drop_buffer = 0.015  # 单位m
    mpp.param_area = 0.0  # 选点打分参数, 越+, 越优先使得待放置物体体积x-y方向延伸后所覆盖的已有物体的体积最大
    mpp.param_touch = 0  # 选点打分参数, 越+, 越优先使得待放置物体和其他已放置物体的侧面接触面积较大
    mpp.param_ignore = 2  # 选点打分参数, 越+, 越不会使得待放置物体下表面一部分悬空
    mpp.param_position = 10  # 选点打分参数, 越+, 放置位置影响的权重就会越大
    mpp.param_corner = 0  # 选点打分参数, 越+, 越优先选择边角，仅码朝外规划在用

    mpp.pallet_expand.extend([0, 0])  # 规划时允许在托盘外x,y方向上延展的最大尺寸, 单位m
    mpp.slide_len.extend([0.15, 0.15, 0.15])  # preplace到place的x, y, z方向上的滑动距离, 单位m
    mpp.touch_tolerance = 0  # 高度允差, 单位m
    mpp.h_tolerance = 0.015  # 设置高度允差, 单位米
    mpp.hull_tolerance.extend([0, 6, 4])
    mpp.min_support_ratio = 0.5  # 物体底面最小支撑面积占比
    mpp.min_corner_support_ratio = 0  # 物体四角被支撑时, 单个角点最小支撑面积占比
    mpp.min_corner_support_len = 0.01  # 物体四角被支撑时, 单个角点支撑正方形最小边长, 单位m
    mpp.direction_weight.extend([1, 1, 100])  # position打分时各轴的权重
    mpp.position_pixel_size.extend([0.01, 0.01, 0.001])  # position打分时各轴的分辨率, 单位m
    mpp.dist_to_max_height = -1  # 当前规划的箱子的底面距离当前垛最大高度的距离阈值, 超出此阈值的候选点将被过滤, 单位米
    mpp.barcode_direction = 0  # 0无要求，1箱子长边朝外，2箱子短边朝外
    mpp.enable_crossover = False  # 是否鼓励当前箱子与下层箱子的边缘交叠
    mpp.edge_overlap_nums = 0  # 当前箱子多少条边需要与下层箱子重合
    mpp.max_stacked_layer = -1  # 最大堆叠高塔的层数, <=0则不限制
    mpp.enable_center_alignment = False  # 是否对准中轴线
    mpp.rotate_90 = False  # 托底式吸盘是否需要旋转90度

    mpp.plan_nums = 1  # 单个物体的规划结果数

    mpp.stable_weight = 1  # 整垛打分权重，稳定性
    mpp.layer_weight = 1  # 整垛打分权重，放置层数
    mpp.local_weight = 1  # 整垛打分权重，单个箱子累积

    sp.num_topk = 5
    sp.num_roll_out = 50
    sp.roll_out_depth = 5
    sp.expand_depth = 0
    sp.time_budget = -1
    sp.pallet_budget = -1

    place_corner_id = 0  # 放置角点

    ################################用户设置#############################

    order = Order[Task](order_id=order_id)
    planner = OfflineMixedTaskPlanner()

    @backup_manager_wrapper()
    def success_callback(planning_result: PlanningResult, **kwargs):
        """规划完成后的回调函数.

        该函数会在规划完成后被调用, 主要功能:
            1. 根据规划结果构造任务并更新至 Order,
            2. 将规划结果推送至 WCS.

        Args:
            planning_result: 规划结果.
                planning_result 有两个属性: pallet_results 和 pallet_num.
        """
        order_done_num = 0
        for comb_index in range(len(planning_result.chosen_comb)):
            num_in_round = 0
            for pallet in planning_result.batch_results[comb_index].pallet_results:
                num_in_round += len(pallet.box_results)

            # 生成任务
            # 任务ID: 订单ID + 整体索引号
            task = Task(
                task_id=f"{order_id}_{comb_index}",
                order_id=order_id,
                task_type=TaskType.MULTI_PAL_OFFLINE,
                target_num=num_in_round,
                order_done_num=order_done_num,
                from_ws=body.from_ws,
                to_ws=body.to_ws,
                customized_data={"sku_pallet": planning_result.sku_pallet},
            )
            order.add(task)
            order_done_num += num_in_round

        # 通知 WCS 混码任务规划完成
        if not body.sync:
            # 异步执行才推送消息, 同步规划的接口在当前接口返回
            report_plan_result(order, result=planning_result)
        mp.order.info(
            _("混码规划订单({0})规划完成，一共创建({1})个任务").format(order_id, order.total_num)
        )

    @backup_manager_wrapper()
    def failure_callback(error: "XYZBaseError", **kwargs):
        """规划失败的回调函数."""
        order_manager.remove(order)
        report_plan_result(order, error=error)
        mp.order.info(_("混码规划订单({0})规划失败，该订单已被删除").format(order_id))

    order_manager.add(order)
    mp.order.info(_("收到混码规划订单({0})，正在规划中...").format(order_id))

    # NOTICE: 混码offline在切换大任务时清空工作空间，依项目而定
    # 设置工作空间状态为未就绪.
    workspace_manager.create_or_modify(ws_id=body.from_ws, is_ready=True)
    workspace_manager.create_or_modify(ws_id=body.to_ws, is_ready=True)

    if body.sync:
        # 同步规划
        try:
            result = planner.plan(
                order_id=order_id,
                mpp=mpp,
                sp=sp,
                box_type=box_type,
                sku_tool_map=sku_tool_map,
                sku_info=body.sku_info,
                max_pallet_num=wcs_settings.offline_order_planning_config.max_pallet_num,
                conversion_dict=wcs_settings.offline_order_planning_config.conversion_dict,
                to_ws=body.to_ws,
                place_corner_id=place_corner_id,
            )
        except Exception as e:
            mp.order.error(_("混码规划订单({0})规划失败").format(order_id))
            # 规划失败, 删除订单
            order_manager.remove(order)
            raise e
        success_callback(planning_result=result)
        # data 返回的内容依项目而定
        return make_json_response(error=0, data=result.dict())
    else:
        # 提交混码规划，后台执行规划.
        planner.submit(
            success_callback=success_callback,
            failure_callback=failure_callback,
            sku_info=body.sku_info,
            max_pallet_num=wcs_settings.offline_order_planning_config.max_pallet_num,
            conversion_dict=wcs_settings.offline_order_planning_config.conversion_dict,
            to_ws=body.to_ws,
            place_corner_id=place_corner_id,
            order_id=order_id,
            mpp=mpp,
            sp=sp,
            box_type=box_type,
            sku_tool_map=sku_tool_map,
        )
        return make_json_response(error=0)


@bp.route("/multi_class_pal_task/start", methods=["POST"])
@req_log(log=wcs_log)
@backup_manager_wrapper()
@validator()
@openapi.api_doc(tags=["WCS", "离线混码"], request_body=GetMultiPalStartInfoSchema)
def multi_class_pal_task_offline_start(body: GetMultiPalStartInfoSchema):
    """开始执行混码offline任务.

    Raises:
        ValidationError: 接口参数输入异常.
        TaskDuplicateError: 传入了重复的任务.

    Returns:
        ---
        {
            "code": 0,
            "msg": "success",
            "data": {}
        }
        ---

    """
    order_id = body.order_id
    order = order_manager.get_order_or_404(order_id)
    round_id = body.round_id
    current_task = order.get_task_or_404(f"{order_id}_{round_id}")
    # 一个订单的第一个任务 round_id 为 0
    if round_id > 0:
        # 检查任务的执行顺序, 必须等待 (round_id - 1) 完成后，才能执行 round_id 任务
        last_task = order.get_task_or_404(f"{order_id}_{round_id - 1}")
        if not last_task.is_ended() or task_manager.is_exists(last_task):
            raise XYZValidationError(
                error_message="任务执行顺序错误, 必须等待上一个任务({0})完成!".format(round_id - 1)
            )

    wcs_log.info(f"Current sub task: {current_task}")

    if current_task.task_status == TaskStatus.FINISHED:
        raise TaskDuplicateError(f"任务({current_task.task_id})已经完成, 请勿重复执行.")

    from apps.models import db
    from apps.ext.offline_mix_planner.model import PlanningResultModel

    results = db.session.query(PlanningResultModel).filter(PlanningResultModel.order_id==order_id, PlanningResultModel.is_deleted==False).first()
    slots_res = []
    pallet_boxes = []
    for round_res in results.result["batch_results"]:
        pallet_res = round_res["pallet_results"]
        if len(pallet_res) == 1:
            # 接着之前继续码, 不换托
            pallet_boxes += pallet_res[0]["box_results"]
        elif pallet_res[0]["box_results"] == []:
            # 第一个是空, 意味着换托, 先把之前的结果加进上一个托盘
            slots_res.append({"box_res": pallet_boxes})
            pallet_boxes = []
            # 中间都是完整新托
            for idx, other_pallet_res in enumerate(pallet_res[1:-1]):
                pallet_boxes = pallet_res[idx]["box_results"]
                slots_res.append({"box_res": pallet_boxes})
                pallet_boxes = []
            # 最后一个不能保证是完整托
            pallet_boxes = pallet_res[-1]["box_results"]
        else:
            # 第一个非空, 接着之前继续码, 不换托
            pallet_boxes += pallet_res[0]["box_results"]
            slots_res.append({"box_res": pallet_boxes})
            pallet_boxes = []
            # 中间都是完整新托
            for other_pallet_res in pallet_res[1:-1]:
                idx = pallet_res.index(other_pallet_res)
                pallet_boxes = pallet_res[idx]["box_results"]
                slots_res.append({"box_res": pallet_boxes})
                pallet_boxes = []
            # 最后一个不能保证是完整托
            pallet_boxes = pallet_res[-1]["box_results"]
    if pallet_boxes != []:
        # 尾料为一托
        slots_res.append({"box_res": pallet_boxes})
    current_task.customized_data.update({"slots_res": slots_res})

    # 加入任务管理器, 等待执行
    task_manager.append(current_task)

    message = _("收到混码任务({0})").format(current_task.task_id)
    mp.order.info(message)
    return make_json_response(error=0)


# -------------------------------------------
# customized wcs
# -------------------------------------------
@bp.route("/clear_order_manager", methods=["POST"])
@req_log(log=wcs_log)
@backup_manager_wrapper()
@validator()
@openapi.api_doc(tags=["WCS", "订单管理"])
def clear_order_manager():
    """清空所有订单

    Returns:
        ---
        {
            "code": 0,
            "msg": "success",
            "data": {}
        }
        ---

    """
    if current_app.status == "stopped":
        order_manager.clear()
        mp.order.info(_("所有订单被清空"))
        return make_json_response(error=0)
    raise XYZBaseError(error_message=_("机器人运行中，订单删除失败"))
