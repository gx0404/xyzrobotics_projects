#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
    wcs_adaptor.piece_picking.wcs
    ~~~~~~~~~~~~~~~~~~~~~

    包含提供给上游WCS的接口

    Copyright (c) XYZ Robotics Inc. - All Rights Reserved
    Unauthorized copying of this file, via any medium is strictly prohibited
    Proprietary and confidential
    Developer: Chen Kun <kun.chen@xyzrobotics.ai>, May, 2022
"""
from flask import Blueprint, request
from marshmallow import Schema, fields

from apps import _, mp, cached_app, openapi, wcs_log
from apps.openapi.responses import JSONResponse
from wcs_adaptor.enums import TaskType
from wcs_adaptor.entity import StrictSkuInfo, Workspace, LiteOrder as Order
from wcs_adaptor.piece_picking.entity import PPTask
from wcs_adaptor.helpers import backup_manager_wrapper
from wcs_adaptor.manager import (
    task_manager,
    order_manager,
    workspace_manager
)
from wcs_adaptor.piece_picking.plc import PlcClient
from wcs_adaptor.piece_picking.schema import (
    RobotMotionSchema,
    StandardOrderSchema,
    NoticeWsSchema
)
from wcs_adaptor.settings import wcs_settings
from wcs_adaptor.piece_picking.utils import catch_request_log

logger = wcs_log
bp = Blueprint('pp_wcs', __name__, url_prefix='/api/piece_picking/wcs')


def standard_response(code=0, message="", data=None, exception=None):
    """返回wcs接口请求的标准响应格式.

    Args:
        code(int): 0为正常，非零为异常。
        message(str): 异常时返回发生的异常信息，如果没有异常，默认为空字符串。
        data(bool): 返回数据请求成功或者失败的状态。
        exception(None or Exception): 如果请求过程中内部程序发生异常，会将
            异常传入该实体中，该实体包含了简要的异常原因。

    Returns:
        response(dict): 返回符合JSON规范的字典数据，用于返回服务器响应信息。
    """
    if exception is None:
        response = {
            "code": code,
            "msg": message,
            "data": data
        }
    else:
        response = {
            "code": -1,
            "msg": str(exception),
            "data": None
        }
    return response


class STDResponseSchema(Schema):
    """标准的返回报文"""
    code = fields.Integer()
    msg = fields.String()
    data = fields.Dict(allow_none=True)

    class Config:
        schema_extra = {
            "example": {
                "code": 0,
                "msg": "success",
                "data": {}
            }
        }


@bp.route("/notice_ws_ready", methods=['POST'])
@catch_request_log(logger=logger, response_func=standard_response)
@backup_manager_wrapper()
@openapi.api_doc(
    tags=["WCS"],
    summary="通知工作空间就绪",
    request_body=NoticeWsSchema,
    response=JSONResponse(model=STDResponseSchema)
)
def notice_ws_ready():
    """更新某个工作空间状态为已就绪。"""
    req_data = request.get_json()
    data, errors = NoticeWsSchema().load(req_data)
    if errors != {}:
        return standard_response(exception=errors)
    workspace_manager.ready(ws_id=data["ws_id"])
    return standard_response()


@bp.route("/notice_ws_release", methods=['POST'])
@catch_request_log(logger=logger, response_func=standard_response)
@backup_manager_wrapper()
@openapi.api_doc(
    tags=["WCS"],
    summary="通知工作空间未就绪",
    request_body=NoticeWsSchema,
    response=JSONResponse(model=STDResponseSchema)
)
def notice_ws_not_ready():
    """更新某个工作空间状态为未就绪。"""
    req_data = request.get_json()
    data, errors = NoticeWsSchema().load(req_data)
    if errors != {}:
        return standard_response(exception=errors)
    workspace_manager.not_ready(ws_id=data["ws_id"])
    return standard_response()


@bp.route("/read_plc", methods=['POST'])
@catch_request_log(logger=logger, response_func=standard_response)
@backup_manager_wrapper()
@openapi.api_doc(tags=["WCS"], repsonse=JSONResponse(model=STDResponseSchema))
def read_plc():
    """从PLC读取地址信号。

    Body:
        ---
        {
            "address": 10,
            "length": 1
        }
        ---
    """
    plc_client = PlcClient()
    value, error_message = plc_client.read(**request.get_json())
    if value is None:
        return standard_response(exception=error_message)
    data = list(value)
    return standard_response(data=data)


@bp.route("/write_plc", methods=['POST'])
@catch_request_log(logger=logger, response_func=standard_response)
@backup_manager_wrapper()
@openapi.api_doc(tags=["WCS"], repsonse=JSONResponse(model=STDResponseSchema))
def write_plc():
    """向PLC写入地址信号。

    Body:
        ---
        {
            "address": 10,
            "value": [1, 0, 0]
        }
        ---
    """
    plc_client = PlcClient()
    error_message = plc_client.write(**request.get_json())
    if error_message:
        return standard_response(exception=error_message)
    return standard_response()


@bp.route("/order", methods=['POST'])
@catch_request_log(logger=logger, response_func=standard_response)
@backup_manager_wrapper()
@openapi.api_doc(tags=["WCS"], request_body=StandardOrderSchema, response=JSONResponse(model=STDResponseSchema))
def receive_order():
    """接收上游WCS接收的订单。"""
    # 校验请求参数
    req_data = request.get_json()
    data, errors = StandardOrderSchema().load(req_data)
    if errors != {}:
        mp.order.error(_("接收的订单校验失败: {0}").format(errors))
        return standard_response(exception=errors)
    # Create task entity list
    tasks = []
    order_id = data["order_id"]
    for idx, item in enumerate(data["tasks"]):
        if item.get("task_id") is None:
            item["task_id"] = f"{order_id}_{idx}"
        sku_info = StrictSkuInfo.parse_obj(item["sku_info"])
        task = PPTask(
            task_id=item["task_id"],
            task_index=idx,
            task_type=TaskType(item['task_type']),
            order_id=order_id,
            sku_info=sku_info,
            target_num=item["target_num"],
            picking_position=item["picking_position"],
            picking_bin=item["picking_bin"],
            placing_position=item["placing_position"],
            placing_bin=item["placing_bin"],
            customized_data=item.get("customized_data")
        )
        tasks.append(task)
    # 创建Order实体
    order = Order(order_id=order_id, tasks=tasks)
    # 检查订单列表中是否已经存在订单实体，如果存在，则报错，并不添加进订单列表中
    # TODO: 如果客户订单ID重复，如何处理订单ID重复的情况?
    if order_manager.is_exists(order):
        output_data = standard_response(
            code=-1,
            message="订单号{0}已经存在!".format(order.order_id),
        )
        mp.order.error(output_data["msg"])
        return output_data
    else:
        order_manager.add(order)
        # Add workspace up the workspace manager
        for task in order.tasks:
            if not workspace_manager.is_exists(task.picking_position):
                workspace_manager.add(Workspace(ws_id=task.picking_position))
            if not workspace_manager.is_exists(task.placing_position):
                workspace_manager.add(Workspace(ws_id=task.placing_position))
        mp.order.info(_("添加订单【{0}】").format(order.order_id))
    return standard_response()


@bp.route("/order", methods=['DELETE'])
@catch_request_log(logger=logger, response_func=standard_response)
@backup_manager_wrapper()
@openapi.api_doc(tags=["WCS", "Manager"], response=JSONResponse(model=STDResponseSchema))
def delete_order():
    """删除订单。

    如果订单存在，继续根据order_id在task_manager中查找任务信息，如果任务存在
    在任务列表中，那么删除该任务；如果需要删除的订单正在被执行，需要先停止执行该订单对应的任务
    才能删除该订单

    Body:
        ---
        {
            "order_id": "order_id"
        }
        ---
    """
    # 校验请求参数
    req_data = request.get_json()
    order_id = req_data.get("order_id")
    # 查找订单是否存在
    if order_id not in order_manager._OrderManager__orders_map:
        return standard_response(
            code=0,
            message=f"订单{order_id}不存在"
        )
    # 查找任务列表中的任务
    remove_task_list = []
    for task in task_manager.tasks:
        if order_id == task.order_id:
            # 检查当前状态是否是正常分拣，如果正在分拣，则不允许删除
            if cached_app().status == 'ready':
                message = _("订单【{0}】正在被执行，不允许删除，如需删除，请先暂停服务").format(task.order_id)
                return standard_response(
                    code=-1,
                    message=message
                )
            else:
                remove_task_list.append(task.task_id)
    # 删除订单和对应的任务
    for task_id in remove_task_list:
        task_manager.remove_task_by_id(task_id)
        message = _("删除订单中的任务【{0}】").format(task_id)
        mp.order.info(message)
    order_manager.remove_by_id(order_id)
    message = _("删除订单【{0}】").format(order_id)
    mp.order.info(message)

    return standard_response()


@bp.route("/order_manager", methods=['GET'])
@catch_request_log(logger=logger, response_func=standard_response)
@backup_manager_wrapper()
@openapi.api_doc(tags=["WCS", "Manager"])
def get_orders_info():
    """获取订单列表中的所有订单信息

    Returns:
        ---
        {
            "code": 0,
            "message": "success",
            "data": [
                {
                    "order_id": "order_id",
                    "create_time": "2020-01-01 00:00:00",
                }
            ]
        }
        ---
    """
    data = []
    for order in order_manager.orders:
        data.append({
            "order_id": order.order_id,
            "created_time": str(order.create_time)
        })
    return standard_response(data=data)


@bp.route("/task_manager", methods=['GET'])
@catch_request_log(logger=logger, response_func=standard_response)
@backup_manager_wrapper()
@openapi.api_doc(tags=["WCS", "Manager"])
def get_tasks_info():
    """获取任务列表的所有任务信息

    Returns:
        ---
        {
            "code": 0,
            "message": "success",
            "data": [
                {
                    "order_id": "order_id",
                    "create_time": "2020-01-01 00:00:00",
                }
            ]
        }
        ---
    """
    data = []
    for task in task_manager.tasks:
        data.append(task.to_wcs_json())
    return standard_response(data=data)


@bp.route("/clear_manager", methods=['POST'])
@catch_request_log(logger=logger, response_func=standard_response, indent=None)
@backup_manager_wrapper()
@openapi.api_doc(tags=["WCS", "Manager"])
def clear_manager():
    """清空当前任务中的所有订单信息

    包括订单列表，任务列表，工作空间列表

    Returns:
        ---
        {
            "code": 0,
            "message": "success",
            "data": {
                "workspace_list": [{
                    "workspace": "workspace",
                    "is_ready": "true"
                }],
                "task_list": [{
                    "task_id": "task_id",
                    "create_time": "2020-01-01 00:00:00",
                }],
                "order_list": [{
                    "order_id": "order_id",
                    "create_time": "2020-01-01 00:00:00",
                }],
            }
        }
        ---
    """
    data = {
        "order_list": [],
        "task_list": [],
        "workspace_list": []
    }
    for workspace in workspace_manager._workspaces:
        data["workspace_list"].append({
            "workspace": workspace.ws_id,
            "is_ready": str(workspace.is_ready)
        })
    for task in task_manager.tasks:
        data["task_list"].append({
            "task_id": task.task_id,
            "created_time": str(task.create_time)
        })
    for order in order_manager.orders:
        data["order_list"].append({
            "order_id": order.order_id,
            "created_time": str(order.create_time)
        })
    # 清空订单列表，任务列表和工作空间列表
    workspace_manager.clear()
    task_manager.clear()
    order_manager.clear()

    message = _("订单列表，任务列表，工作空间列表被清空")
    logger.warn(message)
    mp.order.info(message)

    return standard_response(data=data)


@bp.route("/robot_motion", methods=['POST'])
@catch_request_log(logger=logger, response_func=standard_response, indent=None)
@backup_manager_wrapper()
@openapi.api_doc(tags=["WCS"], request_body=RobotMotionSchema, response=JSONResponse(model=STDResponseSchema))
def robot_motion():
    """控制机器人（实际为XTF）的运动阶段，实现接口禁用和启用"""
    # 校验请求参数
    req_data = request.get_json()
    data, errors = RobotMotionSchema().load(req_data)
    if errors != {}:
        # TODO(kun.chen 2022-11-11)：需要补充该翻译的其他语言
        mp.order.error(_("数据校验失败: {0}").format(errors))
        return standard_response(exception=errors)
    for key, value in data.items():
        wcs_settings.pp_xtf_api_map[key] = value
    
    # # XXX: 自行选择是否需要保存到wcs_adaptor文件中
    # wcs_settings.dumps()
    return standard_response()
