# -*- coding: utf-8 -*-
"""
Copyright (c) XYZ Robotics Inc. - All Rights Reserved
Unauthorized copying of this file, via any medium is strictly prohibited
Proprietary and confidential
Author: Yuhang Wu <yuhang.wu@xyzrobotics.ai>, 2022-05-05

订单相关的接口
"""
import json

from flask import Blueprint

from apps import catch_log, make_json_response, validator, openapi
from apps.ext.offline_mix_planner.utils import get_planning_result_by_order_id
from apps.ext.offline_mix_planner.entity import PlanningResultRecord
from wcs_adaptor.manager import order_manager

bp = Blueprint("order", __name__, url_prefix="/api/manager/order")


@bp.route("/", methods=["GET"])
@catch_log()
@openapi.api_doc(tags=["WCS", "订单管理"], summary="查询 OrderManager 中的所有订单")
def list_orders():
    """获取多个order."""
    data = [
        json.loads(order.json(exclude={"tasks_map"})) for order in order_manager.orders
    ]
    return make_json_response(data=data)


@bp.route("/<order_id>", methods=["GET"])
@catch_log()
@validator()
@openapi.api_doc(tags=["WCS", "订单管理"], summary="根据 order_id 查询订单详情")
def get_order(order_id: str):
    """获取一个订单."""
    order = order_manager.get_order_or_404(order_id)
    return make_json_response(data=json.loads(order.json(exclude={"tasks_map"})))


@bp.route("/<order_id>/planning_result", methods=["GET", "POST"])
@catch_log()
@validator()
@openapi.api_doc(
    tags=["WCS", "离线混码"],
    response_body=PlanningResultRecord,
    summary="根据 order_id 查询该订单的离线混码规划结果",
)
def get_planning_result(order_id: str):
    """根据订单ID获取离线规划结果.

    Args:
        order_id(str): 订单编号
    """
    # NOTICE(YuhangWu):
    # 通过订单号获取规划结果，已由标准库实现，如需个性化定制，请重写此接口函数
    return get_planning_result_by_order_id(order_id)
