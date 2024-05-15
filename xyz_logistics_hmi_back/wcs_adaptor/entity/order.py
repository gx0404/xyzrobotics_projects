#  -*- coding: utf-8 -*-
#  Copyright (c) XYZ Robotics Inc. - All Rights Reserved
#  Unauthorized copying of this file, via any medium is strictly prohibited
#  Proprietary and confidential
#  Author: Yuhang Wu <yuhang.wu@xyzrobotics.ai>, 2022/9/20 下午6:01
from datetime import datetime
from typing import Generic, TypeVar

from pydantic import Field

from apps.ext.datastructs import ABCLiteOrder
from wcs_adaptor.entity.task import LiteBaseTask
from wcs_adaptor.enums import OrderStatus
from wcs_adaptor.exceptions import WCSError

T = TypeVar("T", bound=LiteBaseTask)


class LiteOrder(ABCLiteOrder[T], Generic[T]):
    """轻量订单类.

    Attributes:
        order_id(str): 订单ID.
        order_status(OrderStatus): 订单状态.
        tasks(List[Task]): 任务列表.
        start_time(datetime): 订单开始时间.
        end_time(datetime): 订单结束时间.
        error_code(int): 错误码.
        error_msg(str): 错误信息.
    """

    order_status: OrderStatus = Field(default=OrderStatus.PENDING, description="订单状态")

    def start(self) -> None:
        """开始订单."""
        if self.start_time is None:
            self.order_status = OrderStatus.STARTED
            self.start_time = datetime.now()

    def finish(self, auto_remove: bool = True) -> None:
        """标记订单已完成并从manager中移除订单.

        Args:
            auto_remove(bool): 是否从manager中移除当前订单.
        """
        if self.is_finished():
            return
        self.order_status = OrderStatus.FINISHED
        self.end_time = datetime.now()
        if auto_remove:
            self.remove_self()

    def end(self, auto_remove: bool = True) -> None:
        """设置订单已结束.

        Args:
            auto_remove(bool): 是否从manager中移除当前订单.
        """
        self.order_status = OrderStatus.ENDED
        self.end_time = datetime.now()
        if auto_remove:
            self.remove_self()

    def terminate(self, error: WCSError = None, auto_remove: bool = True) -> None:
        """终止订单并从manager中移除订单.

        Args:
            error(WCSError): WCSError及子类异常对象.
            auto_remove(bool): 是否从manager中移除当前订单.
        """
        if error is None:
            error = WCSError()
        self.error_code = error.error_code
        self.error_msg = error.error_message
        self.end_time = datetime.now()
        self.order_status = OrderStatus.TERMINATED
        if auto_remove:
            self.remove_self()

    def is_pending(self) -> bool:
        return self.order_status == OrderStatus.PENDING

    def is_started(self) -> bool:
        return self.order_status == OrderStatus.STARTED

    def is_finished(self) -> bool:
        """判断订单是否已完成.

        Returns:
            bool: 订单是否已完成.
        """
        return self.order_status == OrderStatus.FINISHED

    def is_ended(self) -> bool:
        """判断订单是否已结束."""
        return self.order_status == OrderStatus.ENDED

    def is_terminated(self) -> bool:
        """判断订单是否终止."""
        return self.order_status == OrderStatus.TERMINATED
