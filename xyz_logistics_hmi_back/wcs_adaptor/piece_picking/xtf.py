#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
    wcs_adaptor.piece_picking.rafcon
    ~~~~~~~~~~~~~~~~~~~~~

    包含rafcon交互接口，如申请任务、反馈分拣结果等功能

    Copyright (c) XYZ Robotics Inc. - All Rights Reserved
    Unauthorized copying of this file, via any medium is strictly prohibited
    Proprietary and confidential
    Developer: Chen Kun <kun.chen@xyzrobotics.ai>, May, 2022
"""
from flask import Blueprint, request
from marshmallow import Schema, fields

from apps.base_app.views.command.command_api import stop_xtf

from apps import _, mp, validate, record_error, cached_app, openapi, xtf_log
from apps.openapi.responses import JSONResponse
from wcs_adaptor.helpers import backup_manager_wrapper
from wcs_adaptor.manager import order_manager, task_manager, workspace_manager
from wcs_adaptor.settings import wcs_settings
from wcs_adaptor.zh_msg import ALL_ERROR, DEFAULT_ERROR
from wcs_adaptor.plc_listener import plc_set_redlight
from wcs_adaptor.piece_picking.schema import (
    GetTaskInfoOutputSchema,
    ReportStepOutcomeSchema,
    ErrorHandleInputSchema
)
from wcs_adaptor.piece_picking.wcs_request import (
    feedback_single_result,
    feedback_task_result,
    feedback_order_result
)
from wcs_adaptor.piece_picking.entity import PPTask
from wcs_adaptor.piece_picking.utils import catch_request_log


logger = xtf_log
bp = Blueprint('pp_xtf', __name__, url_prefix='/api/piece_picking/xtf')


def standard_response(error=0, error_message="", result=True, exception=None):
    """返回xtf接口请求的标准响应格式。

    Args:
        error(int): 0为正常，非零为异常。
        error_message(str): 异常时返回发生的异常信息，如果没有异常，默认为空字符串。
        result(bool): 返回数据请求成功或者失败的状态。

    Returns:
        response(dict): 返回符合JSON规范的字典数据，用于返回服务器响应信息。
    """
    if exception is None:
        response = {
            "error": error,
            "error_message": error_message,
            "result": result
        }
    else:
        response = {
            "error": 1,
            "error_message": str(exception),
            "result": False
        }
    return response


class STDResponseSchema(Schema):
    """标准的返回报文"""
    error = fields.Integer()
    error_message = fields.String()
    result = fields.Boolean()

    class Config:
        schema_extra = {
            "example": {
                "error": 0,
                "error_message": "",
                "result": True
            }
        }


def check_workspace_is_ready_by_task(task: PPTask):
    """检查当前待执行任务的需要的工作空间是否已经就绪，如果未就绪，则返回未就绪的抓取位置列表

    Args:
        task(PPTask): 一个任务实例，包含了任务数据

    Returns:
        list: 返回未就绪的抓取位置列表，一般形式如"1", "2"这样的，特殊情况下可根据项目的实际
            需求在wcs_adaptor.settings的pp_workspace_conversion进行设置对应
    """
    not_ready_list = []
    # 获取当前任务的抓取空间、放置空间
    pick_ws = workspace_manager.get(task.picking_position)
    place_ws = workspace_manager.get(task.placing_position)
    # 检查抓取空间、放置空间是否就绪
    if not pick_ws.is_ready:
        not_ready_list.append(pick_ws.ws_id)
    if not place_ws.is_ready:
        not_ready_list.append(place_ws.ws_id)
    return not_ready_list


def send_log_to_hmi(message: dict, message_type: str = "error", method: int = 0):
    """发送一个信息给HMI，HMI将弹窗显示，通常为错误信息

    Args:
        error(dict): 错误信息
        message_type(str): (可选)信息类型，有info, warning, error三种类型
        method(int): (可选)弹窗警示的附加功能，目前分拣站默认使用0即可，1和2是拆码垛使用


    Returns:
        dict: 推送给HMI前端的格式化信息
    """
    result = {
        "code": message["code"],
        "msg_type": message_type,
        "error_handle_method": method,
        "en_msg": message["error_msg"],
        "zh_msg": message["zh_msg"],
        "zh_tip": message["tip"],
        "ja_msg": message["ja_msg"],
        "ja_tip": message["ja_tip"],
        "class": message.get("class", "motion")
    }
    # 通知HMI弹窗汇报异常
    mp.system.error(result)
    return result


@bp.route('/get_task_info', methods=["POST"])
@catch_request_log(ignoreReq=True, ignoreRes=True, response_func=standard_response, indent=None)
@openapi.api_doc(tags=["XTF"], summary="获取任务信息")
def get_task_info():
    """xtf会向该接口请求分拣任务，获取单个抓取任务及SKU信息。

    Returns:
        ---
        {
            "error": 0,
            "error_message": "",
            "result:": True,
            "task_id": "99999",
            "sku_info": {
                "sku_id": "001",
                "length": 0.1,
                "width": 0.1,
                "height": 0.1,
                "weight": 200,
                "type": "box"
            },
            "num_to_pick": 20,
            "pick_ws": "s_0",
            "place_ws": "t_0",
            "return_ws": "s_0",
            "clear_pick_ws": False,
            "clear_place_ws": False,
            "state": "INIT"
        }
        ---
    """
    # 检查接口是否被禁用
    if wcs_settings.pp_xtf_api_map.get("get_task_info", True) is False:
        return standard_response(
            error_message="接口已被禁用",
            result=False
        )
    # 先检查当前任务列表中的数据是否已经完成，如果未完成，则优先处理未完成的任务
    # 如果已经完成，则需要将已经完成的任务上报给上游WCS，上报完成后从订单管理器
    # 中获取一个最新订单，然后从该订单中解析任务到任务列表中。
    # 从任务列表中，取出当前优先需要执行的任务
    task = None
    while True:
        task = task_manager.first()
        if not task:
            # 当前无任务需要分拣，检查订单列表中是否有需要执行的订单
            order = order_manager.first()
            if not order:
                # 没有待执行的订单，则直接返回结果
                return standard_response(
                    error_message="当前没有需要执行的订单",
                    result=False
                )
            if order.done_num + order.end_num >= order.total_num:
                order_manager.remove(order)
                # 当发生订单中任务为空或者任务列表中没有任务的情况时，该订单不再执行
                # TODO: 直接上报订单分拣结果, 并且删除在列表中的订单
                return standard_response(
                    error_message="订单{0}中没有需要执行的任务".format(order.order_id),
                    result=False
                )
            # 将任务存入订单列表
            for task in order.tasks:
                task_manager.append(task)

            # 重新从任务列表中获取一次任务
            task = task_manager.first()
            if task is None:
                # 检查任务获取成功
                return standard_response(
                    error_message="当前无任务需要分拣",
                    result=False
                )
        if order_manager.get_order_by_id(task.order_id) is None:
            # The order of task has been deleted
            task_manager.remove(task)
        elif task.is_finished():
            # Delete the task if finished
            task_manager.remove(task)
        else:
            break

    not_ready_list = check_workspace_is_ready_by_task(task)
    if not_ready_list:
        # 工作空间未就绪
        return standard_response(
            error_message="抓取放置位置{0}未就绪".format(not_ready_list),
            result=False
        )
    # 正常响应的输出数据
    output_data = {
        "error": 0,
        "error_message": "",
        "result": False,    # 波次信息获取成功为True, 获取失败为False
        "task_id": "",      # 任务编号（最小的任务单位）
        "sku_info": {       # 物料信息（包含机械臂在抓取时需要的信息）
            "sku_id": "",   # 物料ID
            "length": 0.0,  # 物料长度(单位:米)，客户传给我们的单位为毫米
            "width": 0.0,   # 物料宽度(单位:米)，客户传给我们的单位为毫米
            "height": 0.0,  # 物料高度(单位:米)，客户传给我们的单位为毫米
            "weight": 0.0,    # 物料重量(单位:克)
            "type": ""      # 物料类型
        },
        "num_to_pick": 1,   # 抓取数量
        "pick_code": "",    # 抓取空间的条码/二维码
        "pick_ws": "",      # 抓取空间位置
        "place_code": "",   # 抓取空间的条码/二维码
        "place_ws": "",     # 放置空间位置
        "return_ws": "",    # 返回空间位置
    }
    # 当前有任务可以进行
    output_data["result"] = True
    output_data["task_id"] = task.task_id
    output_data["pick_ws"] = task.from_ws()
    output_data["place_ws"] = task.to_ws()
    # TODO: 根据实际情况判断需要返回的空间位置，加快分拣节拍
    # 默认返回抓取空间位置
    output_data["return_ws"] = task.from_ws()
    # SKU物料信息，用于视觉识别
    output_data["sku_info"] = task.sku_info.dict()
    # 需要对物料的长宽高重进行单位换算，任务中物料的长宽高以毫米为单位，
    # 需要转换为以米为单位，响应给xtf
    if "length" in output_data["sku_info"]:
        output_data["sku_info"]["length"] /= 1000.0
    if "width" in output_data["sku_info"]:
        output_data["sku_info"]["width"] /= 1000.0
    if "height" in output_data["sku_info"]:
        output_data["sku_info"]["height"] /= 1000.0
    # order存在且开始时间为None, 则说明当前任务是该订单的第一个任务, 即更新开始时间.
    order = order_manager.get_order_by_id(task.order_id)
    if order and order.start_time is None:
        order.start()
    # 开始任务.
    if task and task.start_time is None:
        task.start()
        message = _("任务【{0}】开始执行，所在订单【{1}】").format(task.task_id, task.order_id)
        mp.order.info(message)

    #############################################################
    # 验证输出数据
    data, errors = GetTaskInfoOutputSchema().dump(output_data)
    if errors != {}:
        return standard_response(exception=errors)

    # # XXX: 在部分项目中，该接口接收到某一次请求后，可能就需要禁用该端口，直到在其他接口实现接口状态的更改
    # wcs_settings.pp_xtf_api_map["get_task_info"] = False
    # # XXX: 保存状态到文件（如果不需要后端启动时依然记录之前的状态，不建议每次保存，也推荐不保存）
    # # 特别是get_task_info接口，由于被xtf调用过于频繁，会占用较多IO资源
    # wcs_settings.dumps()

    return data


@bp.route('/allow_pick', methods=["POST"])
@catch_request_log(response_func=standard_response, indent=None)
@openapi.api_doc(tags=["XTF"], summary="允许机械臂拍照和抓取物体", response=JSONResponse(model=STDResponseSchema))
def allow_pick():
    """允许机械臂拍照和抓取物体。在请求allow_pick接口时，机械臂可能处于机械臂原点上方，
    也可能处于放置位置。

    Body:
        ---
        {
            "task_id": "string",    # 任务ID
        }
        ---
    """
    # 检查接口是否被禁用
    if wcs_settings.pp_xtf_api_map.get("allow_pick", True) is False:
        return standard_response(
            error_message="接口已被禁用",
            result=False
        )
    # TODO: 处理允许机械臂拍照和抓取物体的业务逻辑，如果没有需要处理的业务逻辑，则
    # TODO: result返回True
    req_data = request.get_json()
    task_id = req_data.get("task_id")
    task = task_manager.get_task_by_id(task_id=task_id)
    if task is None:
        return standard_response(
            error_message="当前无任务需要分拣",
            result=False
        )
    not_ready_list = check_workspace_is_ready_by_task(task)
    if not_ready_list:
        # 工作空间未就绪
        return standard_response(
            error_message="工作空间{0}未就绪".format(not_ready_list),
            result=False
        )
    message = _("当前任务【{0}】，机械臂准备抓取物体...").format(task.task_id)
    mp.order.info(message)

    # # XXX: 在部分项目中，该接口接收到某一次请求后，可能就需要禁用该端口，直到在其他接口实现接口状态的更改
    # wcs_settings.pp_xtf_api_map["allow_pick"] = False
    # # XXX: 保存状态到文件（如果不需要后端启动时依然记录之前的状态，不建议每次保存，也推荐不保存）
    # wcs_settings.dumps()

    return standard_response(result=True)


@bp.route('/allow_move', methods=["POST"])
@catch_request_log(response_func=standard_response, indent=None)
@openapi.api_doc(tags=["XTF"], summary="允许机械臂移动到放置位置", response=JSONResponse(model=STDResponseSchema))
def allow_move():
    """允许机械臂移动至放置位置，一般情况下，起始点为抓取位置上方，终止点为放置位置上方。

    Body:
        ---
        {
            "task_id": "string",    # 任务ID
        }
        ---
    """
    # 检查接口是否被禁用
    if wcs_settings.pp_xtf_api_map.get("allow_move", True) is False:
        return standard_response(
            error_message="接口已被禁用",
            result=False
        )
    # TODO: 处理允许机械臂移动至放置位的业务逻辑，如果没有需要处理的业务逻辑，则
    # TODO: result返回True
    req_data = request.get_json()
    task_id = req_data.get("task_id")
    task = task_manager.get_task_by_id(task_id=task_id)
    if task is None:
        return standard_response(
            error_message="当前无任务需要分拣",
            result=False
        )
    not_ready_list = check_workspace_is_ready_by_task(task)
    if not_ready_list:
        # 工作空间未就绪
        return standard_response(
            error_message="工作空间{0}未就绪".format(not_ready_list),
            result=False
        )
    message = _("当前任务【{0}】，机械臂准备移动至放置位置...").format(task.task_id)
    mp.order.info(message)

    # # XXX: 在部分项目中，该接口接收到某一次请求后，可能就需要禁用该端口，直到在其他接口实现接口状态的更改
    # wcs_settings.pp_xtf_api_map["allow_move"] = False
    # # XXX: 保存状态到文件（如果不需要后端启动时依然记录之前的状态，不建议每次保存，也推荐不保存）
    # wcs_settings.dumps()

    return standard_response(result=True)


@bp.route('/allow_release', methods=["POST"])
@catch_request_log(response_func=standard_response, indent=None)
@openapi.api_doc(tags=["XTF"], summary="允许释放机械臂抓具上的物体", response=JSONResponse(model=STDResponseSchema))
def allow_release():
    """允许释放机械臂抓具上的物体。之后机械臂可能会将物体放入放置位置中，也可能会在放置位
    置上方直接释放物体。

    Body:
        ---
        {
            "task_id": "string",    # 任务ID
        }
        ---
    """
    # 检查接口是否被禁用
    if wcs_settings.pp_xtf_api_map.get("allow_release", True) is False:
        return standard_response(
            error_message="接口已被禁用",
            result=False
        )
    # TODO: 处理允许机械臂释放物料到放置位的业务逻辑，如果没有需要处理的业务逻辑，则
    # TODO: result返回True
    req_data = request.get_json()
    task_id = req_data.get("task_id")
    task = task_manager.get_task_by_id(task_id=task_id)
    if task is None:
        return standard_response(
            error_message="当前无任务需要分拣",
            result=False
        )
    not_ready_list = check_workspace_is_ready_by_task(task)
    if not_ready_list:
        # 工作空间未就绪
        return standard_response(
            error_message="工作空间{0}未就绪".format(not_ready_list),
            result=False
        )
    # 推送信息到HMI的订单日志中
    message = _("当前任务【{0}】，机械臂准备移动至放置位置...").format(task.task_id)
    mp.order.info(message)

    # # XXX: 在部分项目中，该接口接收到某一次请求后，可能就需要禁用该端口，直到在其他接口实现接口状态的更改
    # wcs_settings.pp_xtf_api_map["allow_release"] = False
    # # XXX: 保存状态到文件（如果不需要后端启动时依然记录之前的状态，不建议每次保存，也推荐不保存）
    # wcs_settings.dumps()

    return standard_response(result=True)


@bp.route('/report_step_outcome', methods=["POST"])
@catch_request_log(response_func=standard_response, indent=None)
@backup_manager_wrapper()
@openapi.api_doc(
    tags=["XTF"],
    summary="上报执行结果",
    request_body=ReportStepOutcomeSchema,
    response=JSONResponse(model=STDResponseSchema)
)
def report_step_outcome():
    """机械臂完成一次抓取任务后，反馈抓放物体的分拣结果，包含三种情况（正常、满框、空桶），
    其他相关异常，如: 视觉识别失败、多次抓取失败、抓具掉落、气泵开启失败等异常，见error_handle
    接口。

    TODO: return workspace
    """
    # 检查接口是否被禁用
    if wcs_settings.pp_xtf_api_map.get("report_step_outcome", True) is False:
        return standard_response(
            error_message="接口已被禁用",
            result=False
        )

    req_data = request.get_json()
    # 数据有效性校验
    data, errors = ReportStepOutcomeSchema().load(req_data)
    if errors != {}:
        return standard_response(exception=errors)
    # TODO:接收xtf有效请求后，更新分拣数据，并且响应给xtf。处理三种分拣结果的情况
    task = task_manager.get_task_by_id(data["task_id"])
    order = order_manager.get_order_by_id(task.order_id)
    if task is None:
        return standard_response(
            error_message="任务号不存在",
            result=False
        )
    error = data["error"]

    if error == 0:
        # 正常事件
        # 单次任务分拣完成，上报单词抓取结果
        task.report_pick_num(num=1, auto_complete=False)
        # 获取任务完成数量
        done_num = task.done_num
        feedback_single_result(task)
        if task.is_finished():
            if task.is_cycle_mode():
                # 互换抓取空间和放置空间
                message = _("互换抓取空间和放置空间[ {0} <-> {1} ]，并重置任务数量").format(task.from_ws(), task.to_ws())
                mp.order.info(message)
                task.switch_ws()
                # 重置任务数量，并且任务状态切换为pending
                task.reset_done_num()
            else:
                # 设置任务已经完成
                task.finish()
                # 任务完成上报任务分拣结果
                feedback_task_result(task)
        # 推送信息到HMI的订单日志中
        message = _("当前任务【{0}】，任务进度【{1}/{2}】订单进度【{3}/{4}】").format(
            task.task_id,
            done_num,
            task.target_num,
            order.end_num,
            order.total_num
        )
        mp.order.info(message)

        if order and order.done_num >= order.total_num:
            # Using order id but not order entity
            feedback_order_result(order)
            # Remove order if the order finished
            order_manager.remove(order)
            message = _("订单已完成，从订单管理器中删除订单【{0}】").format(order.order_id)
            mp.order.info(message)
    elif error == 1:
        # 当抓取料箱发生空框时，且目标数量为-1，说明任务本身就是要抓空
        # 所以当料箱发生空框，是一种正常现象
        if task.target_num == -1:
            # 任务完成上报任务分拣结果
            feedback_task_result(task)
            # 推送信息到HMI的订单日志中
            message = _("当前任务【{0}】，任务进度【{1}/{2}】，订单进度【{3}/{4}】").format(
                task.task_id,
                task.done_num,
                task.target_num,
                order.end_num,
                order.total_num
            )
            mp.order.info(message)
            # 检查当前任务是否是循环压测模式，当处于循环压测模式时，则抓取料箱到指定数量
            # 或者抓空料箱后，就自动互换放置空间和抓取空间
            if task.is_cycle_mode():
                # 互换抓取空间和放置空间
                message = _("互换抓取空间和放置空间[ {0} <-> {1} ]，并重置任务数量").format(task.from_ws(), task.to_ws())
                mp.order.info(message)
                task.switch_ws()
                task.reset_done_num()
            else:
                # 设置任务已经完成
                task.finish()
            # 检查订单是否完成
            if order and order.done_num + order.end_num >= order.total_num:
                # Using order id but not order entity
                feedback_order_result(order)
                # Remove order if the order finished
                order_manager.remove(order)
                message = _("抓取空间已清空，订单完成，从订单管理器中删除订单【{0}】").format(order.order_id)
                mp.order.info(message)
        else:
            # 推送信息到HMI的订单日志中
            message = _("当前任务【{0}】，机械臂分拣过程发生空框事件，机械臂停止抓取，等待人工处理").format(task.task_id)
            mp.order.error(message)
            # 设置PLC切换灯色为红色
            plc_set_redlight()
            # 通知HMI弹窗汇报异常
            send_log_to_hmi(ALL_ERROR["70019"])
            # 控制机械臂停止
            stop_xtf()
    elif error == 2:
        # 满框事件
        # 推送信息到HMI的订单日志中
        message = _("当前任务【{0}】，机械臂分拣过程发生满框事件，机械臂停止抓取，等待人工处理").format(task.task_id)
        mp.order.error(message)
        # 设置PLC切换灯色为红色
        plc_set_redlight()
        # 通知HMI弹窗汇报异常
        send_log_to_hmi(ALL_ERROR["70018"])
        # 控制机械臂停止
        stop_xtf()
    elif error == 3:
        # 掉落事件
        # 推送信息到HMI的订单日志中
        message = _("当前任务【{0}】，机械臂分拣过程发生掉落事件，机械臂停止抓取，等待人工处理").format(task.task_id)
        mp.order.error(message)
        # 设置PLC切换灯色为红色
        plc_set_redlight()
        # 通知HMI弹窗汇报异常
        send_log_to_hmi(ALL_ERROR["70003"])
        # 控制机械臂停止
        stop_xtf()
    else:
        return standard_response(
            error_message="非法参数!",
            result=False
        )

    # # XXX: 在部分项目中，该接口接收到某一次请求后，可能就需要禁用该端口，直到在其他接口实现接口状态的更改
    # wcs_settings.pp_xtf_api_map["report_step_outcome"] = False
    # # XXX: 保存状态到文件（如果不需要后端启动时依然记录之前的状态，不建议每次保存，也推荐不保存）
    # wcs_settings.dumps()
    
    return standard_response(result=True)


@bp.route('/error_handle', methods=["POST"])
@catch_request_log(response_func=standard_response, indent=None)
@backup_manager_wrapper()
@openapi.api_doc(
    tags=["XTF"],
    summary="机械臂异常处理接口",
    request_body=ErrorHandleInputSchema,
    response=JSONResponse(model=STDResponseSchema)
)
def error_handler():
    """接收机械臂分拣过程中产生的异常信息，如视觉识别失败、多次抓取失败、抓具掉落、气泵开启
    失败等异常。并且在HMI界面中弹窗显示

    Returns:
        standard_response(dict): 给XTF的标准响应

    Raises:
        ValidationError: 参数异常.

    Response:
        standard_response: 给XTF的标准响应

    """
    # 数据有效性校验，如果出错，测响应错误信息
    data = validate(ErrorHandleInputSchema, request.get_json())
    # 停止xtf
    stop_xtf()

    # 从配置中获取异常的所有信息
    error_data = ALL_ERROR.get(data["error"], DEFAULT_ERROR)
    error_data["error_handle_method"] = 0

    # 设置PLC切换灯色
    plc_set_redlight()

    # 通知HMI弹窗汇报异常
    mp.system.error(error_data)

    # 推送给HMI，订单执行过程出现异常需要处理
    message = _("错误码【{0}】任务执行出现异常【{1}】操作提示【{2}】").format(
        error_data["code"],
        error_data["msg"],
        error_data["tip"]
    )

    task = task_manager.first()
    # 记录该异常至数据库
    record_error(
        code=error_data["code"],
        msg=error_data["msg"],
        source="xtf",
        task=task,
        tip=error_data["tip"]
    )
    app = cached_app()
    app.globals.set_rafcon_error({
        "code": error_data["code"],
        "error_msg": data.get("error_msg", None),
    })
    mp.order.error(message)
    # TODO: 根据xtf响应的异常进行响应操作
    return standard_response(result=True)


@bp.route("/error")
@openapi.api_doc(tags=["XTF", "TEST"], summary="用于测试的异常接口", response=JSONResponse(model=STDResponseSchema))
def get_error():
    """返回rafcon的异常状态.

    Notes: 用于自动化测试
    wiki: https://xyz-robotics.atlassian.net/wiki/spaces/depalletizeproduct/pages/538935297/RAFCON+2.0.0#Rafcon-%E5%8D%95%E5%85%83%E6%B5%8B%E8%AF%95%EF%BC%8C%E8%8E%B7%E5%8F%96%E5%BC%82%E5%B8%B8%E4%BF%A1%E6%81%AF

    Returns:
        ---
        {
            "error": 1000,
            "error_message": "this is error message",
            "result": False
        }
        ---
    """
    app = cached_app()
    error = app.globals.get_rafcon_error()
    return standard_response(error=error["code"], error_message=error["error_msg"])
