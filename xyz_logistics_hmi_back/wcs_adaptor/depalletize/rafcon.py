# -*- coding: utf-8 -*-
"""单拆项目 Rafcon 接口"""
from flask import Blueprint, request

from apps import (
    _,
    cached_app,
    make_json_response,
    mp,
    openapi,
    record_error,
    validate,
    validator,
)
from apps.base_app.flask_hook import req_log
from apps.base_app.views.command.command_api import stop_xtf
from apps.log import outside_log, xtf_log
from apps.utils import node
from wcs_adaptor.depalletize.schema import (
    ErrorHandleInputSchema,
    GetSkuInfoInputSchema,
    GetTaskInfoOutputSchema,
    IsWsReadyInputSchema,
    ManualAddBoxInputSchema,
    ReportTaskEndingInputSchema,
    ReportTaskStatusInputSchema,
)
from wcs_adaptor.depalletize.wcs_request import (
    get_sku_info_from_wcs,
    notice_pick_ws_is_empty,
    report_exception,
    turn_agv,
    report_init_error,
    report_order_finish_offline,
    report_task_finish_offline,
    
    notice_pick_complete,    
    
    #拣配任务
    report_depal_action_status,
    notice_depal_place_ws_is_full,
    report_depal_task_finish,
    #合托任务
    report_merge_task_finish,
    #输送线混码任务
    report_multi_pal_action_status,
    report_multi_pal_task_finish,
    notice_multi_pal_place_ws_is_full,
    get_conveyor_tote_type_online,
    #笼车码垛
    report_pallet_action_status,
    report_pallet_task_finish,
    notice_pallet_pal_place_ws_is_full,
    get_conveyor_tote_type_pal,
    
)
from wcs_adaptor.enums import TaskType,TaskStatus
from wcs_adaptor.exceptions import EmptyTaskError, UncompletedTaskError, WCSError
from wcs_adaptor.helpers import backup_manager_wrapper
from wcs_adaptor.manager import order_manager, task_manager, workspace_manager
from wcs_adaptor.plc_listener import plc_set_redlight
from wcs_adaptor.zh_msg import ALL_ERROR, DEFAULT_ERROR

from wcs_adaptor.models import PalletData,CageLayer
from apps.models import db
from datetime import datetime

bp = Blueprint("rafcon", __name__, url_prefix="/api/rafcon")

MAX_NUM = 1  # 机械臂单次抓取所能抓起的最大数量


"""提供网页数据库接口"""
#获取拣配托盘数据
@bp.route("/send_pallet_data", methods=["POST"])
@req_log(log=xtf_log)
@validator()
@openapi.api_doc(tags=["XTF"])
def send_pallet_data():
    """

    获取托盘数据

    """

    model = PalletData.query.first()
    if not model:
        model = PalletData(current_direction="",pallet_tote_data={},pick_tote_data={},\
            cache_pallet_tote_data={},path={},pallet_tote_data_2={},pallet_tote_data_3={}\
            ,pallet_tote_data_7={},pallet_tote_data_8={})
        db.session.add(model)
        db.session.commit()
    pallet_tote_data = model.pallet_tote_data
    for i in range(1,64):
        if str(i) not in pallet_tote_data.keys():
            pallet_tote_data[str(i)] = {
                "barcode":""
            }

    cache_pallet_tote_data = model.cache_pallet_tote_data
    for i in range(1,64):
        if str(i) not in cache_pallet_tote_data.keys():
            cache_pallet_tote_data[str(i)] = {
                "barcode":"",
                "from_pick_id":"",
                "pose_rotation":""
            }    
            

    return make_json_response(error=0, pallet_tote_data=pallet_tote_data,cache_pallet_tote_data=cache_pallet_tote_data)



#初始化的时候闭环异常报错
@bp.route("/init_error", methods=["POST"])
@req_log(log=xtf_log)
@validator()
@openapi.api_doc(tags=["XTF"])
def init_error():
    report_init_error()
    return make_json_response(error=0) 
    

#转向agv
@bp.route("/rafcon_turn_agv", methods=["POST"])
@req_log(log=xtf_log)
@validator()
@openapi.api_doc(tags=["XTF"])
def rafcon_turn_agv():
    data = request.get_json()
    ws_id = data["ws_id"]
    agv_direction = data["agv_direction"] 
    return_data = turn_agv(ws_id,agv_direction).json()
    # import ipdb;ipdb.set_trace()
    error = return_data["error"]
    if error=="0":
        mp.order.info(f"通知wcs转向agv,回报转向中")
        # 获取当前空间对象, 不存在则新增.
        ws = workspace_manager.create_or_modify(ws_id=ws_id)
        # 清空工作空间.
        ws.clear()
        ws.not_ready()
        return make_json_response(error=0) 
    elif error=="1":
        mp.order.info(f"通知wcs转向agv,无需转向")
        return make_json_response(error=1) 
        
    


#清除任务待删除工作空间
@bp.route("/clear_pallet_clear_list", methods=["POST"])
@req_log(log=xtf_log)
@backup_manager_wrapper()
@validator()
@openapi.api_doc(tags=["XTF"])
def clear_pallet_clear_list():
    task = task_manager.first()
    data = request.get_json()
    if task:
        task.pallet_clear_list = data["pallet_clear_list"]
        return make_json_response(error=0,msg=f"已更新任务待删除工作空间{data['pallet_clear_list']}")    
    else:
        return make_json_response(error=0,msg="no task")    


#清空托盘数据
@bp.route("/clear_all_pallet_data", methods=["POST"])
@req_log(log=xtf_log)
@backup_manager_wrapper()
@validator()
@openapi.api_doc(tags=["XTF"])
def clear_all_pallet_data():
    #import ipdb;ipdb.set_trace()
    model = PalletData.query.first()
    if not model:
        model = PalletData(current_direction="",pallet_tote_data={},pick_tote_data={},\
            cache_pallet_tote_data={},path={},pallet_tote_data_2={},pallet_tote_data_3={}\
            ,pallet_tote_data_7={},pallet_tote_data_8={})
        db.session.add(model)
        db.session.commit()
    model.pallet_tote_data = {}
    model.pick_tote_data = {}
    model.cache_pallet_tote_data = {}
    model.path = {}
    model.pallet_tote_data_2 = {}
    model.pallet_tote_data_3 = {}
    model.pallet_tote_data_7 = {}
    model.pallet_tote_data_8 = {}
    db.session.commit()
    return make_json_response(error=0)

#获取拣配托盘数据
@bp.route("/get_pallet_data", methods=["POST"])
@req_log(log=xtf_log)
@backup_manager_wrapper()
@validator()
@openapi.api_doc(tags=["XTF"])
def get_pallet_data():
    """
    获取托盘数据
    """
    try:
        model = PalletData.query.first()
        if not model:
            model = PalletData(current_direction="",pallet_tote_data={},pick_tote_data={},\
                cache_pallet_tote_data={},path={},pallet_tote_data_2={},pallet_tote_data_3={}\
                ,pallet_tote_data_7={},pallet_tote_data_8={})
            db.session.add(model)
            db.session.commit()
        current_direction = model.current_direction
        pallet_tote_data = model.pallet_tote_data
        pick_tote_data = model.pick_tote_data
        cache_pallet_tote_data = model.cache_pallet_tote_data
        path = model.path
        pallet_tote_data_2 = model.pallet_tote_data_2
        pallet_tote_data_3 = model.pallet_tote_data_3    
        pallet_tote_data_7 = model.pallet_tote_data_7
        pallet_tote_data_8 = model.pallet_tote_data_8               
        return make_json_response(error=0, current_direction=current_direction,pallet_tote_data=pallet_tote_data,\
                                pick_tote_data = pick_tote_data,cache_pallet_tote_data=cache_pallet_tote_data,path=path,\
                                pallet_tote_data_2 = pallet_tote_data_2,pallet_tote_data_3 = pallet_tote_data_3,\
                                pallet_tote_data_7 = pallet_tote_data_7,pallet_tote_data_8 = pallet_tote_data_8)
    except:
        return make_json_response(error=-1)

#更新拣配托盘数据
@bp.route("/update_pallet_data", methods=["POST","GET"])
@req_log(log=xtf_log)
@backup_manager_wrapper()
@validator()
@openapi.api_doc(tags=["XTF"])
def update_pallet_data():
    """
    更新托盘数据
    """
    try:
        data = request.get_json()
        model = PalletData.query.first()
        if not model:
            model = PalletData(current_direction="",pallet_tote_data={},pick_tote_data={},\
                cache_pallet_tote_data={},path={},pallet_tote_data_2={},pallet_tote_data_3={}\
                ,pallet_tote_data_7={},pallet_tote_data_8={})
            db.session.add(model)
            db.session.commit()
        if "current_direction" in data.keys():    
            model.current_direction = data["current_direction"]   
        if "pallet_tote_data" in data.keys():
            model.pallet_tote_data = data["pallet_tote_data"]  
        if "pick_tote_data" in data.keys():
            model.pick_tote_data = data["pick_tote_data"]          
        if "cache_pallet_tote_data" in data.keys():
            model.cache_pallet_tote_data = data["cache_pallet_tote_data"]   
        if "path" in data.keys():
            model.path = data["path"]   
        if "pallet_tote_data_2" in data.keys():
            model.pallet_tote_data_2 = data["pallet_tote_data_2"] 
        if "pallet_tote_data_3" in data.keys():
            model.pallet_tote_data_3 = data["pallet_tote_data_3"]    
        if "pallet_tote_data_7" in data.keys():
            model.pallet_tote_data_7 = data["pallet_tote_data_7"] 
        if "pallet_tote_data_8" in data.keys():
            model.pallet_tote_data_8 = data["pallet_tote_data_8"]                                        
        db.session.add(model)
        db.session.commit()
        return make_json_response(error=0)
    except:
        return make_json_response(error=-1)    



#更改历史指令时间
def change_datatime(task:dict):
    for key,item in task.items():
        if type(item)==datetime:
            task[key] = item.strftime('%Y-%m-%d %H:%M:%S')
    return task

#查询上一次任务类型
@bp.route("/get_last_task_type", methods=["POST"])
@req_log(log=xtf_log)
@backup_manager_wrapper()
@openapi.api_doc(tags=["XTF"])
def get_last_task_type(): 
    from wcs_adaptor.models import HistoryTaskModel
    model = HistoryTaskModel.query.all()
    last_task_0 = change_datatime(model[-1].to_dict())
    last_task_1 = change_datatime(model[-2].to_dict())
    output_data = { 
                    "0":last_task_0,
                    "1":last_task_1}
    return output_data      
    
#获取输送线物料信息
@bp.route("/get_conveyor_tote", methods=["POST"])
@req_log(log=xtf_log)
@backup_manager_wrapper()
@openapi.api_doc(tags=["XTF"])
def get_conveyor_tote(): 
    task = task_manager.first()
    if not task:
        return make_json_response(error=1,error_msg="无任务")
    else:
        if task.task_type == TaskType.MULTI_PAL_ONLINE:
            data = get_conveyor_tote_type_online(task).json()
            return make_json_response(error=data["error"],data=data)  
        elif task.task_type == TaskType.SINGLE_PAL:
            data = get_conveyor_tote_type_pal(task).json()
            return make_json_response(error=data["error"],data=data)              
         
#获取输送线物料信息
@bp.route("/pick_complete", methods=["POST"])
@req_log(log=xtf_log)
@backup_manager_wrapper()
@openapi.api_doc(tags=["XTF"])
def pick_complete(): 
    task = task_manager.first()
    if not task:
        return make_json_response(error=1,error_msg="无任务")
    else:
        data = notice_pick_complete(task).json()
        return make_json_response(error=data["error"],data=data)  


@bp.route("/get_task_info", methods=["GET", "POST"])
@req_log(log=xtf_log)
@backup_manager_wrapper()
@validator()
@openapi.api_doc(tags=["XTF"])
def get_task_info():
    """Rafcon 获取单个抓取Cycle任务及SKU信息的接口.

    Notes:
        准确来讲，这个接口是 **开始执行任务** 的接口.
        因为 Rafcon 通过这个接口拿到任务就立即开始执行任务，所以当前接口中涉及 "写" 操作.
        order.start() 和 task.start()

    Returns:
        ---
        {
            "error": 0,
            "error_message": "success",
            "result": True,
            "task_id": "123456",
            "sku_info": {
                "sku_id": "2",
                "length": 0.7,
                "width": 0.4,
                "height": 0.2,
                "weight": 3,
            },
            "max_num": 3,
            "from": "ws1",
            "to": "ws2",
            "task_type": 0,
            "clear_from_ws": False,
            "clear_to_ws": False,
            "undone_num": -1,
            "code": 0,
            "msg": "success",
            "data": {
                "result": True,
                "task_id": "123456",
                "sku_info": {
                    "sku_id": "2",
                    "length": 0.7,
                    "width": 0.4,
                    "height": 0.2,
                    "weight": 3,
                },
                "max_num": 3,
                "from": "ws1",
                "to": "ws2",
                "task_type": 0,
                "clear_from_ws": False,
                "clear_to_ws": False,
                "undone_num": -1
            }
        }
        ---

    Raises:
        ValidationError: 数据校验异常.

    """
    task = task_manager.first()
    # import ipdb;ipdb.set_trace()
    if task:
        if task.is_terminated():
            model = PalletData.query.first()  
            task.finish(auto_remove=False)             
            #拣配任务完成
            if task.task_type == TaskType.SINGLE_DEPAL:
                pallet_tote_data = model.pallet_tote_data
                agv_direction = model.current_direction
                report_depal_task_finish(task,pallet_tote_data,agv_direction,error=98)   
                task.end() 
                clear_all_pallet_data()
            mp.order.error("任务被wcs终止结束")
            output_data = GetTaskInfoOutputSchema()
            data = output_data.dict(by_alias=True)
            return make_json_response(data=data, error=0, **data)
        output_data = GetTaskInfoOutputSchema.parse_obj(task.dict())
        output_data.result = True
        output_data.from_ws = task.from_ws
        output_data.to_ws = task.to_ws

        if task.sku_info:
            output_data.sku_info = output_data.sku_info.dict()
            output_data.sku_info["length"] = task.sku_info.length / 1000
            output_data.sku_info["width"] = task.sku_info.width / 1000
            output_data.sku_info["height"] = task.sku_info.height / 1000
        else:
            output_data.sku_info = {}

        if task.task_type == TaskType.MULTI_DEPAL:  # 混拆项目, 无目标拆垛数
            max_num = MAX_NUM
        else:  # 其他项目, 比较下一次拆(码)允许最大数量
            max_num = (
                min(task.target_num - task.done_num, MAX_NUM)
                if task.target_num > 0
                else MAX_NUM
            )
        output_data.max_num = max_num
        output_data.undone_num = task.target_num - task.done_num
        output_data.order_done_num = task.order_done_num

        pick_ws = workspace_manager.get(task.from_ws)
        output_data.clear_from_ws = pick_ws.is_ready if pick_ws else False

        place_ws = workspace_manager.get(task.from_ws)
        output_data.clear_to_ws = place_ws.is_ready if place_ws else False

        output_data.customized_data = task.customized_data
        output_data.lower_layer = task.lower_layer
        output_data.lower_speed = task.lower_speed

        if task.order_id:
            # order存在且开始时间为None, 则说明当前任务是该订单的第一个任务, 即更新开始时间.
            order = order_manager.get_order_by_id(task.order_id)
            if order and order.is_pending():
                order.start()
                  
        if task.is_pending():
            # 开始任务.
            task.start()
    else:
        output_data = GetTaskInfoOutputSchema()
    data = output_data.dict(by_alias=True)
    
    #获取笼车中欧层数
    model = CageLayer.query.first()
    if not model:
        model = CageLayer(layer_num=6)
        db.session.add(model)
        db.session.commit()
    layer_num = model.layer_num    
    data["layer_num"] = layer_num        
    return make_json_response(data=data, error=0, **data)


@bp.route("/is_pick_ws_ready", methods=["POST"])
@req_log(log=xtf_log)
@openapi.api_doc(tags=["XTF"])
def is_pick_ws_ready():
    """查询抓取工作空间是否就位.

    Body:
        ---
        {
            "task_id": "1",
            "ws_id": "2"
        }
        ---

    Returns:
        ---
        {
            "code": 0,
            "msg": "success",
            "data": {
                "result": True
            },
            "result": True,
            "error": 0,
            "error_message": "success"
        }
        ---

    Raises:
        ValidationError: 数据校验异常.
    """
    # 验证输入数据
    data = validate(IsWsReadyInputSchema, request.get_json())
    ws = workspace_manager.get(ws_id=data.get("ws_id"))
    return make_json_response(data={"result": ws.is_ready}, error=0, result=ws.is_ready)


@bp.route("/is_place_ws_ready", methods=["POST"])
@req_log(log=xtf_log)
@openapi.api_doc(tags=["XTF"])
def is_place_ws_ready():
    """查询放置工作空间是否就位.

    Body:
        ---
        {
            "task_id": "1",
            "ws_id": "2"
        }
        ---

    Returns:
        ---
        {
            "code": 0,
            "msg": "success",
            "data": {
                "result": True
            },
            "result": True,
            "error": 0,
            "error_message": "success"
        }
        ---

    Raises:
        ValidationError: 数据校验异常.
    """
    # 验证输入数据
    data = validate(IsWsReadyInputSchema, request.get_json())
    ws = workspace_manager.get(ws_id=data.get("ws_id"))
    return make_json_response(data={"result": ws.is_ready}, error=0, result=ws.is_ready)

#拣配任务回报笼车放满
@bp.route("/report_depal_full", methods=["POST"])
@req_log(log=xtf_log)
@backup_manager_wrapper()
@validator()
@openapi.api_doc(tags=["XTF"])
def report_depal_full():
    data = request.get_json()
    notice_depal_place_ws_is_full(data["place_id"])
    # 获取工作空间对象.
    ws = workspace_manager.get(ws_id=data["place_id"])
    # 设置当前工作空间状态为未就绪
    ws.not_ready()
    return make_json_response(error=0)   
    
        
#输送线空箱回收任务回报托盘放满
@bp.route("/report_multi_pal_full", methods=["POST"])
@req_log(log=xtf_log)
@backup_manager_wrapper()
@validator()
@openapi.api_doc(tags=["XTF"])
def report_multi_pal_full():
    data = request.get_json()
    notice_multi_pal_place_ws_is_full(data["place_id"])
    # 获取工作空间对象.
    ws = workspace_manager.get(ws_id=data["place_id"])
    # 设置当前工作空间状态为未就绪
    ws.not_ready()
    return make_json_response(error=0)  

#输送线空箱回收任务更新任务状态
@bp.route("/update_multi_task_status", methods=["POST"])
@req_log(log=xtf_log)
@backup_manager_wrapper()
@validator()
@openapi.api_doc(tags=["XTF"])
def update_multi_task_status():
    data = request.get_json()
    task_status = data["task_status"]
    task = task_manager.first()
    if task_status == 13:
        task.ignore_mutation(task.multi_normal)()
    elif task_status == 14:
        task.ignore_mutation(task.multi_cache)()        
    return make_json_response(error=0)  

#获取输送线空箱回收任务任务状态
@bp.route("/get_multi_task_status", methods=["POST"])
@req_log(log=xtf_log)
@backup_manager_wrapper()
@validator()
@openapi.api_doc(tags=["XTF"])
def get_multi_task_status():
    task = task_manager.first()     
    if task:
        return make_json_response(error=0,task_status=task.task_status.value)  
    else:
        return make_json_response(error=0)    

#输送线空箱回收仿真测试
@bp.route("/sim_test", methods=["POST"])
@req_log(log=xtf_log)
@backup_manager_wrapper()
def sim_test():
    """
    更新托盘数据
    """
    try:
        data = request.get_json()
        model = PalletData.query.first()
        if not model:
            model = PalletData(current_direction="",pallet_tote_data={},pick_tote_data={},\
                cache_pallet_tote_data={},path={},pallet_tote_data_2={},pallet_tote_data_3={}\
                ,pallet_tote_data_7={},pallet_tote_data_8={})
            db.session.add(model)
            db.session.commit()
        model.path = {
            "place_id":data["place_id"],
            "tote_type":data["tote_type"],
        }                                       
        db.session.add(model)
        db.session.commit()
        return make_json_response(error=0)
    except:
        return make_json_response(error=-1)   


#笼车空箱回收任务回报托盘放满
@bp.route("/report_pallet_pal_full", methods=["POST"])
@req_log(log=xtf_log)
@backup_manager_wrapper()
@validator()
@openapi.api_doc(tags=["XTF"])
def report_pallet_pal_full():
    data = request.get_json()
    notice_pallet_pal_place_ws_is_full(data["place_id"])
    # 获取工作空间对象.
    ws = workspace_manager.get(ws_id=data["place_id"])
    # 设置当前工作空间状态为未就绪
    ws.not_ready()
    return make_json_response(error=0)  



@bp.route("/report_task_status", methods=["POST"])
@req_log(log=xtf_log)
@backup_manager_wrapper()
@validator()
@openapi.api_doc(tags=["XTF"], request_body=ReportTaskStatusInputSchema)
def report_task_status(body: ReportTaskStatusInputSchema):
    """Rafcon回报任务状态.

    Body:
        ---
        {
            "task_id": "1",                 # 任务编号
            "pick_num": 1,                  # 机械臂本次抓取完成数量
            "drop_num": 1,                  # 机械臂本次发生掉箱的数目
            "is_depal_pallet_empty": False, # 是否拆的托盘已空
            "is_pal_pallet_full": False,    # 是否码的托盘已满
            "error": 0,                     # 错误编号, 0: 成功, 1: 失败
            "error_message": "success"      # 错误消息
        }
        ---

    Returns:
        ---
        {
            "code": 0,
            "msg": "success",
            "data": {},
            "error": 0,
            "error_message": "success"
        }
        ---

    Raises:
        ValidationError: 参数异常.
        TaskNotFoundError: 任务不存在异常.

    """
    data = body
    task = task_manager.get_task_or_404(data.task_id)
    if task.order_id:
        order = order_manager.get_order_or_404(task.order_id)
    else:
        order = None

    if task.is_finished():
        raise WCSError(error_message="任务({0})已完成, 请勿重复调用".format(task.task_id))

    # 抓取计数
    task.report_pick_num(data.pick_num, auto_complete=False)

    # 如果有掉箱，提醒掉箱的数目，主要用于多抓的掉箱处理
    if data.drop_num:
        mp.order.error(
            _("发生掉箱，本次掉箱的数目为: {0}，本次成功抓取的数目为: {1}").format(data.drop_num, data.pick_num)
        )

    # # 回报碼垛盘已满
    # if data.is_pal_pallet_full:
    #     # 回报放置位垛盘已满, 并且会清除放置工作空间就位状态
    #     if task.task_type == TaskType.SINGLE_DEPAL:
    #         notice_depal_place_ws_is_full(body.place_id)
    

    # # 回报拆垛已空
    # if data.is_depal_pallet_empty:
    #     # 回报抓取垛盘已满, 并且会清除放置工作空间就位状态
    #     outside_log.info("report wcs depal pallet empty")
    #     notice_pick_ws_is_empty(task)

    # 上报本次抓取的数量
    if data.pick_num:
        mp.order.info(_("任务进度: {0}/{1}").format(task.done_num, task.target_num))
        #拣配
        if task.task_type == TaskType.SINGLE_DEPAL:
            report_depal_action_status(task, data.pick_num,body.pick_id,body.place_id,\
                body.pick_tote_data,body.place_tote_data)
        #输送线混码
        elif task.task_type == TaskType.MULTI_PAL_ONLINE:
            report_multi_pal_action_status(task, data.pick_num,body.place_id)
        #笼车码垛
        elif task.task_type == TaskType.SINGLE_PAL:
            report_pallet_action_status(task, data.pick_num,body.place_id)            
            


    # 判断任务完成
    #拣配
    if task.task_type == TaskType.SINGLE_DEPAL:
        #目标箱子抓空
        model = PalletData.query.first()
        pick_tote_data = model.pick_tote_data
        cache_pallet_tote_data = model.cache_pallet_tote_data
        pallet_tote_data_2 = model.pallet_tote_data_2
        if not pick_tote_data and not cache_pallet_tote_data and not pallet_tote_data_2:
            task.finish(auto_remove=False)
            mp.order.info(f"目标箱子已抓空,缓存区已还原")
        elif not pick_tote_data and cache_pallet_tote_data and not pallet_tote_data_2:
            task.ignore_mutation(task.depal_finish)()
            mp.order.info(f"目标箱子已抓空,需要还原缓存区箱子")
        elif not pick_tote_data and cache_pallet_tote_data and  pallet_tote_data_2:
            task.ignore_mutation(task.depal_change_180)()
            mp.order.info(f"还原缓存区箱子中,180转向流程")
        task_type = "拣配"   
    #合托        
    elif task.task_type == TaskType.MERGE_TASK:
        model = PalletData.query.first()
        cache_pallet_tote_data = model.cache_pallet_tote_data    
        pallet_tote_data_2 = model.pallet_tote_data_2
        
        if not cache_pallet_tote_data and not pallet_tote_data_2:
            mp.order.info(f"合托任务完成")   
            task.finish(auto_remove=False)  
        task_type = "合托"      
    #输送线混码                        
    elif task.task_type == TaskType.MULTI_PAL_ONLINE:
        task_type = "输送线混码"  
    #笼车码垛
    elif task.task_type == TaskType.SINGLE_PAL:
        task_type = "笼车码垛" 
        if data.is_depal_pallet_empty:
            mp.order.info(f"笼车拆空,笼车码垛任务{task.task_id}完成")   
            task.finish(auto_remove=False)
            
            
        
    
    if task.is_terminated():
        mp.order.info(f"当前{task_type}任务终止完成") 
        is_finished = True
    else:
        is_finished = task.is_finished()    
        
    return make_json_response(
        error=0,
        data={
            "is_task_finished": is_finished,
            "is_order_finished": order.is_finished() if order else False,
        },
    )


@bp.route("/report_task_ending", methods=["POST"])
@req_log(log=xtf_log)
@backup_manager_wrapper()
@validator()
@openapi.api_doc(tags=["XTF"], request_body=ReportTaskEndingInputSchema)
def report_task_ending(body: ReportTaskEndingInputSchema):
    """Rafcon 回报任务结束.

    Rafcon 回报任务结束后，即向 WCS 回报订单结束.

    Raises:
        ValidationError: 参数异常.
        TaskNotFoundError: 任务不存在异常.
    """
    task = task_manager.get_task_or_404(body.task_id)
    model = PalletData.query.first()
    #import ipdb;ipdb.set_trace()
    if body.error == 99:
        mp.order.error(f"扫码失败,结束任务回报异常任务完成")
        task.finish(auto_remove=False)
        #拣配任务完成
        if task.task_type == TaskType.SINGLE_DEPAL:
            pallet_tote_data = model.pallet_tote_data
            agv_direction = model.current_direction
            report_depal_task_finish(task,pallet_tote_data,agv_direction,error=body.error)   
            task.end() 
            clear_all_pallet_data()
            return make_json_response(error=0)
        
    elif body.error == 0:    
        
        if not (task.is_finished() or task.is_terminated()):
            # 任务状态未完成
            raise UncompletedTaskError(task).disable_msg_event()
        #拣配任务完成
        if task.task_type == TaskType.SINGLE_DEPAL:
            pallet_tote_data = model.pallet_tote_data
            agv_direction = model.current_direction
            report_depal_task_finish(task,pallet_tote_data,agv_direction,error=0)   
            task.end() 
            clear_all_pallet_data()
        #合托任务完成    
        elif task.task_type == TaskType.MERGE_TASK:
            pallet_tote_data = model.pallet_tote_data
            report_merge_task_finish(task,pallet_tote_data,error=0)   
            task.end() 
            clear_all_pallet_data() 
        #输送线空箱回收任务完成    
        elif task.task_type == TaskType.MULTI_PAL_ONLINE:
            report_multi_pal_task_finish(task,error=0)   
            task.end() 
            clear_all_pallet_data()     
        #笼车空箱回收任务完成    
        elif task.task_type == TaskType.SINGLE_PAL:
            report_pallet_task_finish(task,error=0)   
            task.end() 
            clear_all_pallet_data()                                     
        return make_json_response(error=0)

@bp.route("error_handle_nostop", methods=["POST"])
@req_log(log=xtf_log)
@backup_manager_wrapper()
@validator()
@openapi.api_doc(tags=["XTF"], request_body=ErrorHandleInputSchema)
def error_handle_nostop(body: ErrorHandleInputSchema):
    error_code = body.data["error_code"]
    error_data = ALL_ERROR.get(error_code, default=DEFAULT_ERROR)
    
    task = task_manager.first()
    if task:
        task_id = task.task_id
        error_handle_method = 1
        task.finish(auto_remove=False)
        task.end() 
        clear_all_pallet_data()  
        # if task.task_type.value in [
        #     TaskType.MULTI_DEPAL.value,
        #     TaskType.SINGLE_DEPAL.value,
        # ]:
        #     error_handle_method = 1
        # elif task.task_type.value in [
        #     TaskType.SINGLE_PAL.value,
        #     TaskType.MULTI_PAL_OFFLINE.value,
        #     TaskType.MULTI_PAL_ONLINE.value,
        # ]:
        #     error_handle_method = 2
        # else:
        #     raise TypeError(f"Unknown task type: {task.task_type}")
    else:
        error_handle_method = 0
        task_id = ""

    clear_all_pallet_data()   
    
    error_data["error_handle_method"] = error_handle_method
    error_data["task_id"] = task_id
    error_data["error_msg"] = body.data["error_msg"]

    # 向WCS报告异常.
    report_exception(error_data)
    
    # PLC 设置红灯亮起
    plc_set_redlight()
    
    # 通知HMI异常内容
    mp.order.error(error_data["msg"])

    # 记录该异常至数据库
    record_error(
        code=error_code,
        msg=error_data["msg"],
        source="xtf",
        task=task,
        tip=error_data["tip"],
    )

    
    return make_json_response(error=0)



@bp.route("error_handle", methods=["POST"])
@req_log(log=xtf_log)
@backup_manager_wrapper()
@validator()
@openapi.api_doc(tags=["XTF"], request_body=ErrorHandleInputSchema)
def error_handler(body: ErrorHandleInputSchema):
    """Rafcon异常回报，并显示到HMI.

    Body:
        ---
        {
            "error": 1,                    # Rafcon 错误编号
            "data": {}                     # 错误参数，在需要自定义消息内容时使用
        }
        ---

    Returns:
        ---
        {
            "code": 0,
            "msg": "success",
            "data": {},
            "error": 0,
            "error_message": "success"
        }
        ---

    Raises:
        ValidationError: 参数异常.

    """
    stop_xtf()
    node.stop_robot_node()
    # import ipdb;ipdb.set_trace()
    error_code = body.data["error_code"]
    error_data = ALL_ERROR.get(error_code, default=DEFAULT_ERROR)

    task = task_manager.first()
    if task:
        task_id = task.task_id
        error_handle_method = 1
        # if task.task_type.value in [
        #     TaskType.MULTI_DEPAL.value,
        #     TaskType.SINGLE_DEPAL.value,
        # ]:
        #     error_handle_method = 1
        # elif task.task_type.value in [
        #     TaskType.SINGLE_PAL.value,
        #     TaskType.MULTI_PAL_OFFLINE.value,
        #     TaskType.MULTI_PAL_ONLINE.value,
        # ]:
        #     error_handle_method = 2
        # else:
        #     raise TypeError(f"Unknown task type: {task.task_type}")
    else:
        task_id = ""
        error_handle_method = 0

    error_data["error_handle_method"] = error_handle_method
    error_data["task_id"] = task_id
    
    # 向WCS报告异常.
    report_exception(error_data)

    # PLC 设置红灯亮起
    plc_set_redlight()

    # 通知HMI异常内容
    mp.order.error(error_data["msg"])

    # 异常弹窗
    mp.system.error(error_data)

    # 记录该异常至数据库
    record_error(
        code=error_code,
        msg=error_data["msg"],
        source="xtf",
        task=task,
        tip=error_data["tip"],
    )

    app = cached_app()
    app.globals.set_rafcon_error(
        {
            "code": error_code,
            "error_msg": body.data.get("error_msg", None),
        }
    )
    return make_json_response(error=0)


@bp.route("/get_sku_info", methods=["POST"])
@req_log(log=xtf_log)
@openapi.api_doc(tags=["XTF"], request_body=GetSkuInfoInputSchema)
def get_sku_info():
    """请求sku资讯 (Rafcon需扫码专用)

    Body:
        ---
        {
            "sku_id": "xx",
            "customized_request": {}
        }
        ---

    Returns:
        如果无type信息，type=-1, 否则根据具体项目需求定义type
        注：type信息有助于告知系统是否需要换夹具
        ---
        {
            "code": 0,
            "msg": "success",
            "data": {
                "sku_info" : {
                    "sku_id": "1",
                    "length": 0.6,
                    "width": 0.5,
                    "height": 0.3,
                    "weight": 2,
                    "type": -1
                }
            },
            "sku_info" : {
                "sku_id": "1",
                "length": 0.6,
                "width": 0.5,
                "height": 0.3,
                "weight": 2,
                "type": -1
            }
        }
        ---

    Raises:
        ValidationError: 参数异常.
    """
    # 验证输入数据
    data = validate(GetSkuInfoInputSchema, request.get_json())
    # Get sku info from wcs
    output_data = get_sku_info_from_wcs(data["sku_id"], data["customized_request"])
    outside_log.debug(output_data)
    return make_json_response(data=output_data, error=0, **output_data)


@bp.route("/manual_add_box", methods=["POST"])
@req_log(log=xtf_log)
@backup_manager_wrapper()
@openapi.api_doc(tags=["XTF"], request_body=ManualAddBoxInputSchema)
def manual_add_box():
    """手动添加箱子.

    Body:
        ---
        {
            "pick_num": 1                  # 机械臂本次抓取完成数量
        }
        ---

    Returns:
        ---
        {
            "code": 0,
            "msg": "success",
            "data": {},
        }
        ---

    Raises:
        TaskNotFoundError: 任务不存在异常.

    """
    # 验证输入数据
    data = validate(ManualAddBoxInputSchema, request.get_json())

    # task = task_manager.first()

    # # 抓取计数
    # if task:
    #     task.report_pick_num(data["pick_num"])
    # else:
    #     raise EmptyTaskError()

    # mp.order.info(_("任务进度: {0}/{1}").format(task.done_num, task.target_num))

    # if task.is_finished():
    #     task.end()
    #     mp.order.info(_("任务已结束"))
    #     report_task_finish(task)

    return make_json_response(error=0,msg="此项目无效")


@bp.route("/error")
@req_log(log=xtf_log)
@openapi.api_doc(tags=["XTF"])
def get_error():
    """返回rafcon的异常状态.

    Notes: 用于自动化测试.
    wiki: https://xyz-robotics.atlassian.net/wiki/spaces/depalletizeproduct/pages/538935297/RAFCON+2.0.0#Rafcon-%E5%8D%95%E5%85%83%E6%B5%8B%E8%AF%95%EF%BC%8C%E8%8E%B7%E5%8F%96%E5%BC%82%E5%B8%B8%E4%BF%A1%E6%81%AF

    Returns:
        ---
        {
            "rafcon_error": True,
            "error_data": {
                "code": "10001",
                "msg_type": "error",
                "en_msg": "Vision Failure",
                "zh_msg": "视觉错误",
                "zh_tip": "重启xvf",
                "ja_msg": "",
                "ja_tip": "",
                "class": "vision"
            },
            "error": 0,
            "error_message": "",
            "code": 0,
            "msg": "success",
            "data": {
                "rafcon_error": True,
                "error_data": {
                    "code": "10001",
                    "msg_type": "error",
                    "en_msg": "Vision Failure",
                    "zh_msg": "视觉错误",
                    "zh_tip": "重启xvf",
                    "ja_msg": "",
                    "ja_tip": "",
                    "class": "vision"
                }
            }
        }
        ---
    """
    app = cached_app()
    rafcon_error = app.globals.get_rafcon_error()
    data = {"error_data": rafcon_error, "rafcon_error": bool(rafcon_error)}
    return make_json_response(data=data, error=0, **data)
