from typing import TypeVar, Union

from apps.exceptions import XYZIntegrityError, XYZNotFoundError
from apps.ext import ABCTaskManager, ABCOrderManager, ABCWorkspaceManager
from apps.ext.datastructs.task import ABCLiteTask
from apps.ext.datastructs.order import ABCLiteOrder
from apps.ext.datastructs.workspace import BaseWorkspace
from apps.utils.lazy import LazyLoader
from wcs_adaptor.entity import Workspace, LiteOrder as Order
from wcs_adaptor.exceptions import (
    ExcessTaskError,
    OrderNotFoundError,
    TaskNotFoundError,
    WorkspaceNotFoundError,
    WorkspaceDuplicateError,
    TaskDuplicateError,
    OrderDuplicateError,
)
from wcs_adaptor.settings import wcs_settings

TT = TypeVar("TT", bound=ABCLiteTask)
OT = TypeVar("OT", bound=ABCLiteOrder)
WT = TypeVar("WT", bound=BaseWorkspace)


class TaskManager(ABCTaskManager[TT]):
    """任务管理器

    基本使用手册: https://xyz-robotics.atlassian.net/wiki/spaces/depalletizeproduct/pages/570523658/Manager#TaskManager
    如需自定义任何方法与属性，在下方正常编写即可.
    """

    def append(self, task: TT) -> None:
        """添加新的任务

        Args:
            task: 任务对象

        Raises:
            TaskDuplicateError: 任务已存在.
            ExcessTaskError: 任务数量超过队列大小限制.
        """
        if task_manager.is_exists(task):
            raise TaskDuplicateError(task_id=task.task_id)
        if self.current_num == self.capacity:
            raise ExcessTaskError(self.capacity)
        return super(TaskManager, self).append(task)

    def get_task_or_404(self, v: Union[int, str]) -> TT:
        """获取任务，如果不存在则抛出异常.

        Args:
            v(str or int): 任务ID或者下标号.

        Returns:
            TaskType: 任务对象.

        Raises:
            TypeError: 参数类型错误.
            TaskNotFoundError: 任务不存在.
        """
        try:
            return super(TaskManager, self).get_task_or_404(v)
        except XYZNotFoundError as err:
            raise TaskNotFoundError(str(v)) from err


class OrderManager(ABCOrderManager[OT]):
    """订单管理器

    基本使用手册: https://xyz-robotics.atlassian.net/wiki/spaces/depalletizeproduct/pages/570523658/Manager#OrderManager
    如需自定义任何方法与属性，在下方正常编写即可.
    """

    def add(self, order: OT) -> None:
        try:
            return super(OrderManager, self).add(order)
        except XYZIntegrityError as err:
            raise OrderDuplicateError(order.order_id) from err

    def get_order_or_404(self, v: Union[int, str]) -> OT:
        """根据订单号返回订单, 若订单不存在则抛出异常.

        Args:
            v(int or str): 订单号或下标索引.

        Raises:
            OrderNotFoundError: 订单不存在异常.
            TypeError: 类型错误.

        Returns:
            Order: 根据订单号返回订单, 若订单不存在则返回None.

        """
        try:
            return super(OrderManager, self).get_order_or_404(v)
        except XYZNotFoundError as err:
            raise OrderNotFoundError(str(v)) from err


class WorkspaceManager(ABCWorkspaceManager[WT]):
    """工作空间管理器

    基本使用手册: https://xyz-robotics.atlassian.net/wiki/spaces/depalletizeproduct/pages/570523658/Manager#WorkspaceManager
    如需自定义任何方法与属性，在下方正常编写即可.
    """

    def add(self, ws: WT) -> WT:
        try:
            return super(WorkspaceManager, self).add(ws)
        except XYZIntegrityError as err:
            raise WorkspaceDuplicateError(ws.ws_id) from err

    def get_workspace_or_404(self, ws_id: str) -> WT:
        """根据空间ID获取工作空间对象.

        如果不存在则返回 None.

        Args:
            ws_id(str): 工作空间ID.

        Returns:
            workspace(Workspace or None): 工作空间对象

        Raises:
            NotFoundWorkspaceError: 未找到该工作空间时, 抛出此异常.
        """
        try:
            return super(WorkspaceManager, self).get_workspace_or_404(ws_id)
        except XYZNotFoundError as err:
            raise WorkspaceNotFoundError(ws_id) from err


kw = wcs_settings.task_manager_config.dict() if wcs_settings else {}
workspace_manager = WorkspaceManager[Workspace]()  # type: ignore


class TaskManagerLazyLoader(LazyLoader):
    """任务管理器懒加载器"""

    def __init__(self):
        """初始化"""
        from apps import settings

        if settings.PROJECT_TYPE == "dpt":
            from wcs_adaptor.depalletize.entity import DPTTask
            self._proxy = TaskManager[DPTTask](**kw)
        elif settings.PROJECT_TYPE == "pp":
            from wcs_adaptor.piece_picking.entity import PPTask
            self._proxy = TaskManager[PPTask](**kw)


class OrderManagerLazyLoader(LazyLoader):
    """订单管理器懒加载器"""

    def __init__(self):
        """初始化"""
        from apps import settings

        if settings.PROJECT_TYPE == "dpt":
            from wcs_adaptor.depalletize.entity import DPTTask
            self._proxy = OrderManager[Order[DPTTask]]()
        elif settings.PROJECT_TYPE == "pp":
            from wcs_adaptor.piece_picking.entity import PPTask
            self._proxy = OrderManager[Order[PPTask]]()


##### NOTICE #####
# 为了使编辑器能够正确进行代码提示, 请根据实际项目类型手动注释
# 默认使用的是拆码垛项目的管理器
from wcs_adaptor.depalletize.entity import DPTTask

task_manager: TaskManager[DPTTask] = TaskManagerLazyLoader()  # type: ignore
order_manager: OrderManager[Order[DPTTask]] = OrderManagerLazyLoader()  # type: ignore

# 如果是分拣站项目，请打开下方注释，并注释上方代码
# from wcs_adaptor.piece_picking.entity import PPTask
# task_manager: TaskManager[PPTask] = TaskManagerLazyLoader()  # type: ignore
# order_manager: OrderManager[Order[PPTask]] = OrderManagerLazyLoader()  # type: ignore
