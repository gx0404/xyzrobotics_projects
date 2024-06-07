# -*- coding: utf-8 -*-
"""
Copyright (c) XYZ Robotics Inc. - All Rights Reserved
Unauthorized copying of this file, via any medium is strictly prohibited
Proprietary and confidential
Author: Yuhang Wu <yuhang.wu@xyzrobotics.ai>, 2022-04-24
"""
import typing as t

from apps import XYZBaseError
from apps.exceptions import XYZValidationError
from apps.enums import EventType

if t.TYPE_CHECKING:
    from wcs_adaptor.entity import LiteBaseTask


class WCSError(XYZBaseError):
    error_code: int = 12000  # wcs及子类异常状态码范围为 12000 ~ 13000
    error_message: str = "Unknown WCS Error."
    msg_event = EventType.ORDER_INFO

    def __repr__(self):
        return self.error_message

    def __str__(self):
        return self.error_message


ValidationError = XYZValidationError


class TaskNotFoundError(WCSError):
    error_code = 12002
    name = "Task Not Found Error"

    def __init__(self, task_id: str):
        self.error_message = f"task_id: {task_id}, 未找到此任务."


class WorkspaceNotFoundError(WCSError):
    error_code = 12003

    def __init__(self, ws_id: str):
        self.error_message = f"workspace_id: {ws_id}, 未找到此工作空间."


class PalletDuplicateError(WCSError):
    error_code = 12004

    def __init__(self, pallet_id: str):
        self.error_message = f"pallet_id: {pallet_id}, 已存在."


class WorkspaceDuplicateError(WCSError):
    error_code = 12005

    def __init__(self, ws_id: str):
        self.error_message = f"workspace_id: {ws_id}, 已存在."


class TaskDuplicateError(WCSError):
    error_code = 12006
    name = "Task Duplicate Error"

    def __init__(self, task_id: str):
        self.task_id = task_id
        self.error_message = f"task_id: {task_id}, 已存在."


class OrderDuplicateError(WCSError):
    error_code = 12007
    name = "Order Duplicate Error"

    def __init__(self, order_id: str):
        self.error_message = f"order_id: {order_id}, 已存在."


class OrderNotFoundError(WCSError):
    error_code = 12008
    name = "Order Not Found Error"

    def __init__(self, order_id: str):
        self.error_message = f"order_id: {order_id}, 不存在."


class PalletNotFoundError(WCSError):
    error_code = 12009
    name = "Pallet Not Found Error"

    def __init__(self, pallet_id: str):
        self.error_message = f"pallet_id: {pallet_id}, 不存在"


class VisionError(WCSError):
    error_code = 12010
    error_message = name = "Vision Error"


class EmptyTaskError(WCSError):
    error_code = 12011
    name = "Empty Task Error"
    error_message = "could not get task because of the task_manager is empty."


class LoadingManagerError(WCSError):
    error_code = 12012
    name = "Loading Manager Failed"

    def __init__(self, e: Exception):
        self.error_message = f"Could not load manager: {repr(e)}"


class ExcessTaskError(WCSError):
    error_code = 12013
    name = "Excess Task Error"

    def __init__(self, size):
        self.error_message = f"The max size of the task manager is {size}, current size is {size}. At first you should complete some of the tasks."


class UncompletedTaskError(WCSError):
    """任务未完成"""
    error_code = 12014
    name = "Uncompleted Task Error"

    def __init__(self, task: "LiteBaseTask"):
        self.error_message = "已完成数量小于预期数量, 当前进度: {}/{}".format(task.done_num, task.target_num)


class InvalidOperationError(WCSError):
    """无效操作"""
    error_code = 12015
    name = "Invalid InvalidOperation Error"

    def __init__(self, error_message: str = ""):
        self.error_message = error_message
