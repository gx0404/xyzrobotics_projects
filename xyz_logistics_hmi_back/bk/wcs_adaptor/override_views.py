#  -*- coding: utf-8 -*-
#  Copyright (c) XYZ Robotics Inc. - All Rights Reserved
#  Unauthorized copying of this file, via any medium is strictly prohibited
#  Proprietary and confidential
#  Author: Yuhang Wu <yuhang.wu@xyzrobotics.ai>, 2022/11/15 上午10:27
#
# Example: 重写视图函数
# 如果标准库中的函数不能满足需求, 可以重写视图函数
# 重写视图函数的方法是使用 `@app.override_view_func` 装饰器, 其用法与 `@app.route` 装饰器一致
# 第一个参数需要指定完整的接口路径，通过 `xlhb routes` 命令查看所有视图函数的路径
# $ xlhb routes
# Endpoint                                Methods           Rule
# --------------------------------------  ----------------  ---------------------------------------------
# cmd.remove_task                         GET, POST         /api/cmd/remove_task
# 其中 `/api/cmd/remove_task` 就是接口路径

from apps import _, cached_app, make_json_response, mp, catch_log, openapi
from apps.log import outside_log
from wcs_adaptor.helpers import backup_manager_wrapper
from wcs_adaptor.manager import order_manager, task_manager

app = cached_app()


@app.override_view("/api/cmd/remove_task", methods=["GET", "POST"])
@catch_log()
@backup_manager_wrapper()
@openapi.api_doc(tags=["WCS", "订单管理"], summary="清空当前任务和订单")
def remove_task():
    """结束并清除现有的任务"""
    task_manager.terminate(error="触发清空任务")
    task_manager.clear()
    order_manager.terminate(error="触发清空订单")
    order_manager.clear()
    outside_log.info("Remove all task")
    mp.order.info(_("已清空任务"))
    return {"error": 0, "code": 0, "msg": "", "error_message": "", "data": {}}


@app.override_view("/api/query/has_unfinished_task", methods=["GET", "POST"])
@catch_log()
@openapi.api_doc(tags=["WCS", "订单管理"], summary="查询当前是否有任务或订单")
def has_unfinished_task():
    """查询是否有未完成的任务
    
    Returns:
        ---
        {
            "code": 0,
            "msg": "success",
            "data": 1
        }
        ---
    """
    if task_manager.first() or order_manager.first():
        return make_json_response(data=1)  # has unfinished task
    else:
        return make_json_response(data=0)  # no unfinished task
