#  -*- coding: utf-8 -*-
#  Copyright (c) XYZ Robotics Inc. - All Rights Reserved
#  Unauthorized copying of this file, via any medium is strictly prohibited
#  Proprietary and confidential
#  Author: Yuhang Wu <yuhang.wu@xyzrobotics.ai>, 2022/8/31 下午3:12
import abc
from datetime import datetime
from typing import Dict, Optional

from pydantic import BaseModel, Field, root_validator, validator

from apps.ext.datastructs.sku import BaseSKU
from apps.ext.signals import (
    task_abort,
    task_done,
    task_ended,
    task_pick,
    task_reset,
    task_start,
)


class ABCLiteTask(BaseModel, abc.ABC):
    """简化版任务抽象类.

    Warnings:
        task_id、order_id、done_num、create_time、start_time、end_time
        以上字段在初始化后不可直接修改, 比如下方的代码是被禁止的:
            task.done_num = 10

    Attributes:
        task_id: 任务ID
        order_id: 订单ID
        sku_info: 物料信息
        target_num: 目标数量
        done_num: 已完成数量
        customized_data: 自定义数据
        create_time: 创建时间
        start_time: 开始时间
        end_time: 结束时间
        error_code: 错误码
        error_msg: 错误信息
    """

    task_id: str = Field(description="任务ID", allow_mutation=False)
    order_id: Optional[str] = Field(
        default=None, description="订单ID", allow_mutation=False
    )
    sku_info: Optional[BaseSKU] = Field(default=None, description="sku信息")
    done_num: int = Field(default=0, description="已完成的数量", allow_mutation=False)
    target_num: int = Field(description="预估数量")
    customized_data: Dict = Field(default_factory=dict, description="自定义字典")
    start_time: Optional[datetime] = Field(
        default=None, description="开始时间", allow_mutation=False
    )
    end_time: Optional[datetime] = Field(
        default=None, description="结束时间", allow_mutation=False
    )
    create_time: datetime = Field(
        default_factory=datetime.now, description="创建时间", allow_mutation=False
    )
    error_code: Optional[int] = Field(default=None, description="异常码")
    error_msg: Optional[str] = Field(default=None, description="异常信息")
    # 用于保存 allow_mutation=False 的字段名.
    _not_mutation_fields = set()
    # 允许修改的方法和信号映射表
    _allow_mutation_methods = {
        "start": task_start,
        "finish": task_done,
        "terminate": task_abort,
        "end": task_ended,
        "report_pick_num": task_pick,
        "reset": task_reset,
    }

    class Config:
        validate_assignment = True

    @root_validator()
    def fill_not_mutation_fields(cls, values):
        """填充不可修改的字段."""
        for _, field_info in cls.__dict__["__fields__"].items():
            if field_info.field_info.allow_mutation is False:
                cls._not_mutation_fields.add(field_info)
        return values

    @validator("customized_data", pre=True)
    def validate_customized_data(cls, value: Optional[Dict]) -> Dict:
        """校验自定义数据."""
        return {} if value is None else value

    @abc.abstractmethod
    def start(self):
        """开始任务"""
        raise NotImplementedError

    @abc.abstractmethod
    def finish(self, auto_remove: bool = True):
        """完成任务"""
        raise NotImplementedError

    @abc.abstractmethod
    def end(self, auto_remove: bool = True):
        """完成任务"""
        raise NotImplementedError

    @abc.abstractmethod
    def terminate(self, error):
        """中止任务"""
        raise NotImplementedError

    @abc.abstractmethod
    def report_pick_num(self, num: int, **kwargs):
        """上报拣货数量"""
        raise NotImplementedError

    @abc.abstractmethod
    def reset(self):
        """重置任务"""
        raise NotImplementedError

    def send_signal(self, signal):
        """用于发送信号的装饰器"""

        def decorator(func):
            def wrapper(*args, **kwargs):
                result = func(*args, **kwargs)
                signal.send(self, **kwargs)
                return result

            return wrapper

        return decorator

    def ignore_mutation(self, func):
        """暂时屏蔽 allow_mutation=False 的限制."""

        def wrapper(*args, **kwargs):
            # 记录变更前的状态
            before_status = {}
            for field in self._not_mutation_fields:
                before_status[field] = field.field_info.allow_mutation
                field.field_info.allow_mutation = True
            result = func(*args, **kwargs)
            # 恢复原来的状态
            for field, status in before_status.items():
                field.field_info.allow_mutation = status
            return result

        return wrapper

    def remove_self(self):
        """移除自身"""
        # 解决循环导入
        from apps.ext.manager import TaskManager

        tm = TaskManager._get_subinstance()
        if tm is None:
            raise RuntimeError("TaskManager 未初始化")
        tm.remove(self)

    def __getattribute__(self, item):
        # 当调用这些方法时, 会触发相对应的信号.
        # 当调用这些方法时，暂时屏蔽 allow_mutation=False 的限制.
        ignore_mutation = object.__getattribute__(self, "ignore_mutation")
        send_signal = object.__getattribute__(self, "send_signal")
        method = object.__getattribute__(self, item)
        _allow_mutation_methods = object.__getattribute__(
            self, "_allow_mutation_methods"
        )
        if item in _allow_mutation_methods:
            method = ignore_mutation(method)
            return send_signal(_allow_mutation_methods[item])(method)
        return super().__getattribute__(item)

