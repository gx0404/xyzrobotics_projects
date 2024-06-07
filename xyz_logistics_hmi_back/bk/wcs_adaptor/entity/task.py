#  -*- coding: utf-8 -*-
#  Copyright (c) XYZ Robotics Inc. - All Rights Reserved
#  Unauthorized copying of this file, via any medium is strictly prohibited
#  Proprietary and confidential
#  Author: Yuhang Wu <yuhang.wu@xyzrobotics.ai>, 2022/9/20 下午6:01
from datetime import datetime
from typing import Optional

from pydantic import Field, validator

from apps.ext.datastructs import ABCLiteTask
from wcs_adaptor.entity import StrictSkuInfo
from wcs_adaptor.enums import TaskStatus, TaskType
from wcs_adaptor.exceptions import ValidationError, WCSError


class LiteBaseTask(ABCLiteTask):
    """轻量任务类.

    该类是 `ABCLiteTasl` 的具体实现，用于描述一个任务的基本属性，如任务ID，任务类型，任务状态等。
    针对不同的任务类型，可以继承该类，添加任务类型特有的属性，例如：拆码垛任务需要抓取位ID，放置位ID等, 分拣站任务需要格口ID等。

    Warnings:
        该类及所有子类不应该被赋值表达式修改，否则会出现不可预知的错误。
        task.task_id = 2, 这是被禁止的，也是不符合逻辑的，task_id 应该是只读属性，不应该被修改。

        1. 修改任务的完成数量，应该使用 `report_pick_num` 方法，而不是直接修改 `done_num` 属性。
        task.report_pick_num() or task.pick()
        2. 完成任务，应该使用 `finish` 方法，而不是直接修改 `task_status` 属性。
            因为除了修改 `task_status` 属性，还需要做一些其他的事情，例如：从任务管理器中移除任务等。
        task.finish()

    Examples:
        >>> task = LiteBaseTask(task_id="123", target_num=2, task_type=TaskType.SINGLE_DEPAL)
        >>> task.task_id
        '123'
        >>> task.task_type = TaskType.MULTI_DEPAL
        Traceback (most recent call last):
        ...
        TypeError: "task_type" has allow_mutation set to False and cannot be assigned
        >>> task.task_status = TaskStatus.FINISHED
        Traceback (most recent call last):
        ...
        TypeError: "task_status" has allow_mutation set to False and cannot be assigned
        >>> task.start()
        >>> task.report_pick_num(num=2, auto_complete=False)
        >>> task.done_num
        2
        >>> task.task_status
        <TaskStatus.STARTED: 1>
        >>> task.finish()
        >>> task.task_status
        <TaskStatus.FINISHED: 2>

    Attributes:
        task_id(str): 任务ID.
        order_id(str): 订单ID, 默认为空.
        task_type(TaskType): 任务类型.
        task_status(TaskStatus): 任务状态.
        start_time(datetime): 任务开始时间.
        end_time(datetime): 任务结束时间.
        error_code(int): 错误码.
        error_msg(str): 错误信息.
        done_num(int): 已完成数量.
        target_num(int): 理论数量.
        sku_info(StrictSkuInfo): sku信息.
        customized_data(dict): 自定义数据.

    Methods:
        report_pick_num: 报告抓取数量
        is_finished: 判断任务是否已完成
        start: 开始任务
        finish: 完成任务
        terminate: 终止任务
        reset: 重置任务状态
    """

    sku_info: Optional[StrictSkuInfo] = Field(default=None, description="sku信息")
    task_status: TaskStatus = Field(
        default=TaskStatus.PENDING, title="任务状态", allow_mutation=False
    )
    task_type: TaskType = Field(title="任务类型", allow_mutation=False)

    @validator("target_num")
    def check_target_num(cls, value):
        """检查目标数量是否合法.

        target_num == -1 时表示无限制.
        target_num > 0 时表示限制数量.
        """
        if value == 0 or value < -1:
            raise ValidationError("target_num is incorrect.")
        return value

    def report_pick_num(
        self, num: int = 1, auto_complete: bool = True, auto_remove: bool = True
    ) -> None:
        """报告本次已抓取的数量.

        默认情况下, 当已完成数量等于理论数量时, 此任务将自动调用 finish 方法.
        可根据 auto_complete 参数来决定是否自动调用 finish 方法.

        Args:
            num(int): 数量, 默认为1.
            auto_complete(bool): 是否自动完成, 默认为True.
            auto_remove(bool): 当 `auto_complete` 为 True 时有效, 用于决定是否将任务从队列中自动移除, 默认为 True.
        """
        ret = self.done_num + num
        if self.task_type == TaskType.MULTI_PAL_OFFLINE:
            self.order_done_num += num
        # -1是特殊情况，不用校验.
        if self.target_num != -1 and ret >= self.target_num:
            self.task_status = TaskStatus.FINISHED
            if auto_complete:
                self.finish(auto_remove=auto_remove)
        self.done_num = ret

    def is_pending(self) -> bool:
        return self.task_status == TaskStatus.PENDING

    def is_started(self) -> bool:
        return self.task_status == TaskStatus.STARTED

    def is_finished(self) -> bool:
        """判断当前任务是否已完成.

        Examples:
            if task.is_finished():
                print("ok")
            else:
                print("failed")

        Returns:
            bool: True已完成, False未完成

        """
        return self.task_status == TaskStatus.FINISHED

    def is_ended(self) -> bool:
        """判断当前任务状态是否为已结束."""
        return self.task_status == TaskStatus.ENDED

    def is_terminated(self) -> bool:
        """判断当前任务状态是否为已终止."""
        return self.task_status == TaskStatus.TERMINATED

    def start(self) -> None:
        """开始任务."""
        self.task_status = TaskStatus.STARTED
        self.start_time = datetime.now()

    def depal_finish(self) -> bool:
        self.task_status = TaskStatus.DEPALFINISHED

    def depal_change_180(self) -> bool:
        self.task_status = TaskStatus.CHANGE_180
            
    def finish(self, auto_remove: bool = True) -> None:
        """标记该任务已完成并从manager中移除任务.

        Args:
            auto_remove(bool): 是否从manager中移除当前任务.

        """
        if self.is_finished():
            return
        self.task_status = TaskStatus.FINISHED
        self.end_time = datetime.now()
        if auto_remove:
            self.remove_self()

    def end(self, auto_remove: bool = True) -> None:
        """标记该任务已结束并从manager中移除任务.

        Args:
            auto_remove(bool): 是否从manager中移除当前任务.

        """
        self.task_status = TaskStatus.ENDED
        self.end_time = datetime.now()
        if auto_remove:
            self.remove_self()

    def terminate(self, error: WCSError = None, auto_remove: bool = True) -> None:
        """终止任务并从manager中移除任务.

        Args:
            error(WCSError): WCSError及子类异常对象.
            auto_remove(bool): 是否从manager中移除当前任务.

        """
        if error is None:
            error = WCSError()
        self.error_code = error.error_code
        self.error_msg = error.error_message
        self.end_time = datetime.now()
        self.task_status = TaskStatus.TERMINATED
        if auto_remove:
            self.remove_self()

    def reset(self):
        """重置任务状态."""
        self.task_status = TaskStatus.PENDING
        self.done_num = 0
        self.error_code = 0
        self.error_msg = ""
        self.start_time = None
        self.end_time = None
