#  -*- coding: utf-8 -*-
#  Copyright (c) XYZ Robotics Inc. - All Rights Reserved
#  Unauthorized copying of this file, via any medium is strictly prohibited
#  Proprietary and confidential
#  Author: Yuhang Wu <yuhang.wu@xyzrobotics.ai>, 2022/9/20 上午11:14
import typing as t

import blinker

from apps.log import hmi_log
from apps.settings import settings

if t.TYPE_CHECKING:
    from apps.ext.datastructs.order import ABCLiteOrder
    from apps.ext.datastructs.task import ABCLiteTask
    from apps.ext.manager.manager import OrderManager

    OT = ABCLiteOrder[ABCLiteTask]
    MT = OrderManager[OT]


def get_order_manager() -> t.Optional["MT"]:
    from apps.ext.manager.manager import OrderManager

    return OrderManager._get_subinstance()


# 任务开始信号
task_start = blinker.Signal("task_start")

# 任务抓取信号
task_pick = blinker.Signal("task_pick")


@task_pick.connect
def task_pick_callback(task: "ABCLiteTask", **kwargs):
    """完成一次抓取的回调"""
    auto_complete = kwargs.get("auto_complete", True)
    if auto_complete and task.target_num != -1 and task.done_num >= task.target_num:
        auto_remove = kwargs.get("auto_remove", True)
        task.finish(auto_remove=auto_remove)
    hmi_log.info("Task({})触发抓取信号".format(task.task_id))


# 任务完成信号
task_done = blinker.Signal("task_done")


@task_done.connect
def task_done_callback(task: "ABCLiteTask", **kwargs):
    """任务完成回调"""
    print("{}, 任务已完成".format(task.task_id))
    if settings.PROJECT_TYPE == "pp":
        task.end(**kwargs)


# 任务中止信号
task_abort = blinker.Signal("task_abort")

# 任务结束信号
task_ended = blinker.Signal("task_ended")


@task_ended.connect
def task_ended_callback(task: "ABCLiteTask", **kwargs):
    """任务结束后的回调"""
    from apps import cached_app, db
    from wcs_adaptor.models import HistoryTaskModel

    auto_remove = kwargs.get("auto_remove", True)

    if task.order_id:
        # 判断该任务所属订单是否满足完成条件
        om = get_order_manager()
        if om:
            order = om.get_order_or_404(task.order_id)
            if order.done_num >= order.total_num:
                order.finish(auto_remove=bool(auto_remove))
            elif settings.PROJECT_TYPE == "pp" and order.end_num >= order.total_num:
                order.finish(auto_remove=bool(auto_remove))
    # 持久化任务
    with cached_app().app_context():
        model = HistoryTaskModel.from_entity(task)  # type: ignore
        db.session.add(model)
        db.session.commit()

    hmi_log.info("已触发任务结束信号")


# 重置任务信号
task_reset = blinker.Signal("task_reset")

# 订单开始信号
order_start = blinker.Signal("order_start")

# 添加任务信号
order_add_task = blinker.Signal("order_add_task")

# 订单完成信号
order_done = blinker.Signal("order_done")


@order_done.connect
def order_done_callback(order: "ABCLiteOrder", **kwargs):
    """订单完成回调"""
    auto_remove = kwargs.get("auto_remove", True)
    if settings.PROJECT_TYPE == "pp":
        order.end(auto_remove=bool(auto_remove))
    print("{}, 订单已完成".format(order.order_id))


# 订单中止信号
order_abort = blinker.Signal("order_abort")

# 订单结束信号
order_ended = blinker.Signal("order_ended")


@order_ended.connect
def order_ended_callback(order: "ABCLiteOrder", **kwargs):
    """订单结束状态的回调"""
    from apps import cached_app, db
    from wcs_adaptor.models import HistoryOrderModel

    # 持久化订单
    with cached_app().app_context():
        model = HistoryOrderModel.from_entity(order)
        db.session.add(model)
        db.session.commit()
