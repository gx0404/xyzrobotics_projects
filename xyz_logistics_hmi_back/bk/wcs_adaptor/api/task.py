# -*- coding: utf-8 -*-
"""
Copyright (c) XYZ Robotics Inc. - All Rights Reserved
Unauthorized copying of this file, via any medium is strictly prohibited
Proprietary and confidential
Author: Yuhang Wu <yuhang.wu@xyzrobotics.ai>, 2022-05-05

任务相关的接口
"""
import json

from flask import Blueprint

from apps import catch_log, make_json_response, wcs_log, openapi
from wcs_adaptor.manager import task_manager

bp = Blueprint("task", __name__, url_prefix="/api/manager/task")


@bp.route("/", methods=["GET"])
@catch_log(log=wcs_log)
@openapi.api_doc(tags=["WCS", "任务管理"], summary="查询 TaskManager 中的所有任务")
def list_task():
    """返回task_manager中的任务."""
    tasks = task_manager.tasks
    data = [json.loads(task.json()) for task in tasks]
    return make_json_response(data=data)


@bp.route("/<task_id>", methods=["GET"])
@catch_log(log=wcs_log)
@openapi.api_doc(tags=["WCS", "任务管理"], summary="根据 task_id 查询任务详情")
def get_task(task_id: int):
    """提供task_id, 返回该任务数据."""
    task = task_manager.get_task_or_404(task_id)
    return make_json_response(data=json.loads(task.json()))

