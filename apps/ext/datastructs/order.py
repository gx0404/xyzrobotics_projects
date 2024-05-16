#  -*- coding: utf-8 -*-
#  Copyright (c) XYZ Robotics Inc. - All Rights Reserved
#  Unauthorized copying of this file, via any medium is strictly prohibited
#  Proprietary and confidential
#  Author: Yuhang Wu <yuhang.wu@xyzrobotics.ai>, 2022/8/31 下午3:15
import abc
from datetime import datetime
from typing import Callable, Dict, Generic, List, Optional, TypeVar, Union

from pydantic import Field, root_validator, validator
from pydantic.generics import GenericModel

from apps.exceptions import XYZBaseError, XYZNotFoundError
from apps.ext.datastructs.task import ABCLiteTask
from apps.ext.signals import (
    order_abort,
    order_add_task,
    order_done,
    order_ended,
    order_start,
)

_T = TypeVar("_T", bound=ABCLiteTask)


class ABCLiteOrder(GenericModel, Generic[_T], abc.ABC):
    """简化版订单抽象类

    Attributes:
        order_id(str): 订单ID.
        tasks(List[Task]): 任务列表.
        tasks_map(Dict[str, Task]): 任务字典, 以任务ID为key.
        start_time(datetime): 订单开始时间.
        end_time(datetime): 订单结束时间.
        error_code(int): 错误码.
        error_msg(str): 错误信息.
        _not_mutation_fields(List[Field]): 不允许变更的字段列表.
    """

    order_id: str = Field(description="订单号", allow_mutation=False)
    tasks: List[_T] = Field(default_factory=list, description="任务列表集合")
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
    _not_mutation_fields: set = set()
    # 允许变更的字段值的方法名与信号映射关系.
    _allow_mutation_methods = {
        "start": order_start,
        "finish": order_done,
        "end": order_ended,
        "terminate": order_abort,
        "add": order_add_task,
    }
    # "任务列表字典集合"
    _tasks_map: Dict[str, _T] = {}

    class Config:
        validate_assignment = True
        fields = {"tasks_map": {"exclude": True}}

    @property
    def tasks_map(self) -> Dict[str, _T]:
        """任务字典, 以任务ID为key."""
        if not self._tasks_map:
            object.__setattr__(
                self, "_tasks_map", {task.task_id: task for task in self.tasks}
            )
        return self._tasks_map

    # @root_validator(pre=True)
    # def validate_tasks(cls, values: Dict) -> Dict:
    #     """校验任务列表."""
    #     tasks = values["tasks"]
    #     if tasks is not None:
    #         tasks_map = {task["task_id"]: task for task in tasks}
    #         values["tasks_map"] = tasks_map
    #     return values

    @root_validator()
    def fill_not_mutation_fields(cls, values) -> Dict:
        """填充不可修改的字段."""
        for _, field_info in cls.__dict__["__fields__"].items():
            if field_info.field_info.allow_mutation is False:
                cls._not_mutation_fields.add(field_info)
        return values

    @validator("customized_data", pre=True)
    def validate_customized_data(cls, value: Optional[Dict]) -> Dict:
        """校验自定义数据."""
        return {} if value is None else value

    def add(self, task: _T) -> None:
        """添加任务.

        Args:
            task(ABCLiteTask): 任务对象.
        """
        self.tasks.append(task)
        self._tasks_map[task.task_id] = task

    @property
    def done_num(self) -> int:
        """获取已完成任务数量.

        Returns:
            int: 已完成任务数量.
        """
        return sum(bool(task.is_finished()) for task in self.tasks)

    @property
    def end_num(self) -> int:
        return sum(bool(task.is_ended()) for task in self.tasks)

    @property
    def total_num(self) -> int:
        """获取任务总数.

        Returns:
            int: 任务总数.
        """
        return len(self.tasks)

    def get_task_by_index(self, index: int) -> _T:
        """根据下标获取任务对象.

        Args:
            index(int): 下标

        Raises:
            IndexError: 下标越界时抛出异常.

        Returns:
            TaskInfo: 任务对象.
        """
        return self.tasks[index]

    def get_task_by_id(self, task_id: str) -> Optional[_T]:
        """根据任务ID获取任务对象.

        Args:
            task_id(str): 任务ID.

        Returns:
            Task: 任务对象, 不存在时返回None.
        """
        return self.tasks_map.get(task_id, None)

    def get_task_or_404(self, v: Union[str, int]) -> _T:
        """根据task_id或索引获取任务.

        如果未获取到任务, 则抛出异常.

        Args:
            v(str or int): 当为int类型时, 使用下标查找, 当为str类型时, 则使用id查找.

        Raises:
            TaskNotFoundError: 任务不存在时抛出.
            IndexError: 下标索引越界时抛出.
            TypeError: v的数据类型不正确时抛出.

        Returns:
            Task: 任务对象.
        """
        if isinstance(v, str):
            task = self.get_task_by_id(v)
        elif isinstance(v, int):
            task = self.get_task_by_index(v)
        else:
            raise TypeError("v must be a string or integer.")

        if task is None:
            raise XYZNotFoundError(error_message=str(v))
        return task

    @abc.abstractmethod
    def start(self) -> None:
        """开始任务."""
        raise NotImplementedError

    @abc.abstractmethod
    def finish(self, auto_remove: bool = True) -> None:
        """完成订单并出从manager中移除.

        Args:
            auto_remove(bool): 默认True, 是否从manager中移除.
        """
        raise NotImplementedError

    @abc.abstractmethod
    def end(self, auto_remove: bool = True) -> None:
        """设置订单已结束.

        Args:
            auto_remove(bool): 是否从manager中移除当前订单.
        """
        raise NotImplementedError

    @abc.abstractmethod
    def terminate(
        self, error: Optional[XYZBaseError] = None, is_remove: bool = True
    ) -> None:
        """终止订单并从manager中移除.

        Args:
            error(WCSError): WCSError及子类异常对象.
            is_remove(bool): 默认True, 是否从manager移除.
        """
        raise NotImplementedError

    def send_signal(self, signal) -> Callable:
        """用于发送信号的装饰器"""

        def decorator(func):
            def wrapper(*args, **kwargs):
                result = func(*args, **kwargs)
                # signal 仅接收一个位置参数，如果需要将参数传递给信号，必须通过 k-v 的形式声明参数
                signal.send(self, **kwargs)
                return result

            return wrapper

        return decorator

    def ignore_mutation(self, func) -> Callable:
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

    def remove_self(self) -> None:
        """从manager中移除自身."""
        # 解决循环导入
        from apps.ext.manager import OrderManager

        om = OrderManager._get_subinstance()
        if om is None:
            raise RuntimeError("OrderManager 未初始化")
        om.remove(self)

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
            return ignore_mutation(send_signal(_allow_mutation_methods[item])(method))
        return super().__getattribute__(item)
