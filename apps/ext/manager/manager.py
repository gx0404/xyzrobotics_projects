import pickle
import threading
import warnings
from functools import wraps
from typing import (
    Dict,
    List,
    Optional,
    Union,
    Generic,
    TypeVar,
    Callable,
    Type,
    get_args,
)

from apps.log import hmi_log
from apps.exceptions import XYZBaseError, XYZIntegrityError, XYZNotFoundError
from apps.ext.manager.storage import MemoryStore, BaseStore, get_store_cls
from apps.ext.datastructs.task import ABCLiteTask
from apps.ext.datastructs.order import ABCLiteOrder
from apps.ext.datastructs.workspace import BaseWorkspace

OrderType = TypeVar("OrderType", bound=ABCLiteOrder)
TaskType = TypeVar("TaskType", bound=ABCLiteTask)
WorkspaceType = TypeVar("WorkspaceType", bound=BaseWorkspace)


class ThreadingLock(object):
    """线程锁"""

    def __init__(self, name: str):
        """

        Args:
            name: 锁名
        """
        self.name = name
        self.lock = threading.RLock()
        super(ThreadingLock, self).__init__()

    def __enter__(self):
        self.lock.acquire()
        hmi_log.debug(f"locked {self.name}")

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.lock.release()
        hmi_log.debug(f"unlock {self.name}")


_OrderManagerLock = ThreadingLock(name="order_manager_lock")
_TaskManagerLock = ThreadingLock(name="task_manager_lock")
_WorkspaceManagerLock = ThreadingLock(name="workspace_manager_lock")


def with_lock(func: Callable):
    """为可调用对象加上线程索."""

    @wraps(func)
    def wrapper(self: object, *arg, **kwargs):
        if isinstance(self, OrderManager):
            _lock = _OrderManagerLock
        elif isinstance(self, TaskManager):
            _lock = _TaskManagerLock
        elif isinstance(self, WorkspaceManager):
            _lock = _WorkspaceManagerLock
        else:
            raise TypeError("未知的Manager类型.")

        with _lock:
            return func(self, *arg, **kwargs)

    return wrapper


class OrderManager(Generic[OrderType]):
    """订单管理类.

    Examples:

        # 创建订单管理器
        >>> order_manager = OrderManager()
        # 添加任务
        >>> order_manager.add(Order(order_id="xxx", ...))
        # 获取第一个任务.
        >>> task = order_manager.first()
        # 获取管理器中的所有订单
        >>> orders = order_manager.orders

    Methods:

        orders: 订单列表, 该对象是内部对象的备份.
        first: 返回第一个订单.
        finish: 完成订单.
        is_exists: 判断订单是否存在.
        add: 新增一个订单.
        append: 新增一个订单.
        get_order_by_id: 通过订单号返回订单.
        get_order_by_index: 通过索引号返回订单.
        remove: 移除订单.
        remove_by_id: 通过订单号移除订单.
        remove_by_index: 通过索引下标移除订单.
        terminate: 终止订单.
        clear: 清空管理器.
        dumps: 序列化管理器.
        loads: 反序列化管理器.

    .. versionchanged:: 1.4.0
        移除 ``OrderManager.__OrderManager_orders`` 和 ``OrderManager.__OrderManager_orders_map`` 私有属性.
    """

    __original_class__: Type[OrderType] = None  # type: ignore
    __sub_instances__ = []

    def __new__(cls, *args, **kwargs):
        if not cls.__sub_instances__:
            cls.__sub_instances__.append(super().__new__(cls))
        return cls.__sub_instances__[0]

    def __init__(self):
        self.__store = get_store_cls()()

    @classmethod
    def _get_subinstance(cls) -> Optional["OrderManager[OrderType]"]:
        """获取子类实例.

        Returns:
            OrderType: 工作空间管理器.
        """
        return cls.__sub_instances__[0] if cls.__sub_instances__ else None

    @property
    def orders(self) -> List[OrderType]:
        """返回订单列表.

        Returns:
            orders(List[Order]): 订单列表集合的备份.

        """
        return self.__store.all()

    def first(self) -> Optional[OrderType]:
        """返回第一个订单, 如果不存在则返回None."""
        order = self.__store.top()
        return None if order is None else order

    def is_exists(self, order: OrderType) -> bool:
        """判断订单是否存在."""
        return order.order_id in self.__store

    @with_lock
    def add(self, order: OrderType) -> None:
        """新增订单.

        Args:
            order(Order): 订单对象.

        Raises:
            XYZIntegrityError: 重复订单.

        """
        if self.is_exists(order):
            raise XYZIntegrityError(
                error_message=f"Order({order.order_id}) already exists"
            )
        self.__store.set(order.order_id, order)

    append = add

    def is_empty(self) -> bool:
        """判断manager是否为空."""
        return len(self.__store) == 0

    def get_order_by_id(self, order_id: str) -> Optional[OrderType]:
        """根据订单号返回订单, 若订单不存在则返回None.

        Args:
            order_id(str): 订单号.

        Returns:
            Optional[Order]: 根据订单号返回订单, 若订单不存在则返回None.

        """
        order = self.__store.get(order_id)
        return None if order is None else order

    def get_order_or_404(self, v: Union[int, str]) -> OrderType:
        """根据订单号返回订单, 若订单不存在则抛出异常.

        Args:
            v(int or str): 订单号或下标索引.

        Raises:
            XYZNotFoundError: 订单不存在异常.
            TypeError: 类型错误.

        Returns:
            Order: 根据订单号返回订单, 若订单不存在则返回None.

        """
        if isinstance(v, str):
            order = self.get_order_by_id(v)
        # elif isinstance(v, int):
        #     order = self.get_order_by_index(v)
        else:
            raise TypeError("v must be a string or integer.")
        if order is None:
            raise XYZNotFoundError(error_message=f"Order({v}) not found")
        return order

    def get_order_by_index(self, index: int) -> Optional[OrderType]:
        """根据索引获取订单.

        索引越界时, 返回None.

        .. deprecated:: 1.4.0
            已废弃, 请使用 ``get_order_by_id`` 方法.

        Args:
            index(int): 索引下标.

        Returns:
            Optional[Order]: 根据下标索引返回订单, 若订单不存在则返回None.
        """
        warnings.warn("已废弃, 请使用 ``get_order_by_id`` 方法.")
        try:
            return self.__orders[index]  # type: ignore
        except IndexError:
            return None

    @with_lock
    def remove(self, order: OrderType) -> None:
        """从管理器中移除订单.

        Args:
            order(Order): 订单对象.

        """
        if self.is_exists(order):
            self.__store.remove(key=order.order_id)

    @with_lock
    def remove_by_index(self, index: int):
        """根据索引下标移除订单.

        .. deprecated:: 1.4.0
            已废弃, 请使用 ``remove_by_id`` 或 ``remove`` 方法.

        Args:
            index(int): 下标索引.
        """
        warnings.warn("已废弃, 请使用 ``remove_by_id`` 或 ``remove`` 方法.")
        if order := self.get_order_by_index(index):
            self.remove(order)

    @with_lock
    def remove_by_id(self, order_id: str) -> None:
        """根据订单号移除订单.

        Args:
            order_id(str): 订单号.
        """
        if order_id in self.__store:
            self.__store.remove(key=order_id)

    def finish(self, order: OrderType):
        """标记订单已完成."""
        order.finish()
        self.remove(order)

    def terminate(self, error: Union[str, XYZBaseError]):
        """终止所有订单.

        Args:
            error: 异常消息 或 XYZBaseError及子类对象.

        """
        if isinstance(error, str):
            error = XYZBaseError(error_message=error)
        for order in self.__store.all():
            order.terminate(error)

    def clear(self):
        """清空订单."""
        self.__store.clear()

    def dumps(self) -> bytes:
        """导出订单.

        Returns:
            bytes: 序列化后的二进制对象.

        """
        # TODO(YuhangWu): 当存储引擎为 Mongo 时，调用此方法时给出警告，此方法不应该在 Mongo 存储引擎中被调用。
        # 获取订单类
        order_cls = self.__get_orig()
        # 通过检查 Order 含有 `_order` 属性来判断是否是 1.4.0 及之前版本的订单类
        if hasattr(order_cls, "_tasks_map"):
            # Order 与 Task 的类是动态创建的，直接使用 pickle.dumps 会因为找不到类而报错
            # 所以将 Order 与 Task 转为字典再序列化
            data = {}
            for order_id, order in self.__store.getdict().items():
                order_dict = order.dict()
                order_dict["tasks"] = [task.dict() for task in order.tasks]
                order_dict.pop("tasks_map", None)
                data[order_id] = order_dict
        else:
            # Order 可能是 1.4.0 版本之前的订单类
            data = self.__store
        return pickle.dumps(data)

    def loads(self, data: bytes):  # type: ignore
        """加载任务.

        Args:
            data: 二进制对象.

        """
        # TODO(YuhangWu): 当存储引擎为 Mongo 时，调用此方法时给出警告，此方法不应该在 Mongo 存储引擎中被调用。
        try:
            data: dict = pickle.loads(data)
            if "_OrderManager__orders_map" in data:
                # 兼容 V1.3.* 及之前版本
                for task_id, task in data.pop(
                    "_OrderManager__orders_map"
                ).items():
                    self.__store.set(task_id, task)
            elif isinstance(data, MemoryStore):
                # 兼容 V1.4.* 版本
                self.__store = data
            else:
                self.__store = get_store_cls()()
                Order = self.__get_orig()
                d = {}
                for order_id, order_dict in data.items():
                    d[order_id] = Order(**order_dict)
                self.__store._data = d
        except Exception as e:
            raise XYZBaseError(error_message="Loading Manager Failed") from e

    def __get_orig(self) -> Type[OrderType]:
        """根据泛型参数获取订单类.

        Returns:
            Type[T]: 类对象.
        """
        if self.__original_class__ is None:
            try:
                self.__original_class__ = get_args(self.__orig_class__)[0]  # type: ignore
            except IndexError:
                cls_name = self.__class__.__name__
                raise RuntimeError(
                    f"请在 {cls_name} 初始化时指定 {cls_name.replace('Manager', '')}"
                    f" 类. 例如: \nmanager = {cls_name}[{cls_name.replace('Manager', '')}]()"
                ) from None
        return self.__original_class__  # type: ignore


class TaskManager(Generic[TaskType]):
    """任务管理类

    这是一个抽象类，需要继承后才能使用。

    Methods:
        append(task): 新增任务对象.
        remove(task): 移除任务对象.
        finish(task): 标记任务已完成.
        is_exists(task): 判断任务是否已存在.
        get_task_by_index(index): 根据下标获取任务对象.
        get_task_by_id(id): 根据任务ID获取任务对象.
        dumps(): 导出任务.
        loads(bytes): 加载任务.
        clear(): 清空任务.
        stop(): 终止任务.

    .. versionchanged:: 1.4.0
        1. 请使用 ``TaskManager.capacity`` 属性代替 ``TaskManager.size`` 属性.
        2. 移除 ``TaskManager.__TaskManager_tasks`` 和 ``TaskManager.__TaskManager_tasks_map`` 私有属性.

    """

    __original_class__: Type[TaskType] = None  # type: ignore
    __sub_instances__ = []

    def __new__(cls, *args, **kwargs):
        if not cls.__sub_instances__:
            cls.__sub_instances__.append(super().__new__(cls))
        return cls.__sub_instances__[0]

    def __init__(self, size: int = -1):
        """按照字典和列表进行存储，方便根据ID和下标进行访问.

        Args:
            size(int): task_manager的大小, 决定其能容量多少个task.
        """
        self.__store = get_store_cls()()
        self.terminate_task: bool = False
        self.__size = size

    @classmethod
    def _get_subinstance(cls) -> Optional["TaskManager[TaskType]"]:
        """获取子类实例.

        Returns:
            TaskManager: 任务管理器对象.
        """
        return cls.__sub_instances__[0] if cls.__sub_instances__ else None

    @property
    def tasks(self) -> List[TaskType]:
        """返回任务列表.

        Returns:
            List[Task]: 返回一批任务对象.
        """
        return self.__store.all()

    @property
    def capacity(self) -> int:
        """task manager容量大小."""
        return self.__size

    def set_capacity(self, size: int):
        """修改容量大小."""
        self.__size = size

    @property
    def current_num(self) -> int:
        """当前任务数量."""
        return len(self.__store)

    def is_empty(self) -> bool:
        """判断manager是否为空."""
        return self.current_num == 0

    def is_full(self) -> bool:
        """判断manager是否已满."""
        return self.capacity == self.current_num

    @with_lock
    def append(self, task: TaskType) -> None:
        """添加新的任务

        Args:
            task: 任务对象

        Raises:
            TaskDuplicateError: 任务已存在.

        """
        if self.is_exists(task):
            raise XYZIntegrityError(
                error_message=f"Task({task.task_id}) already exists"
            )
        self.__store.set(task.task_id, task)

    add = append

    @with_lock
    def remove(self, task: TaskType) -> None:
        """移除任务

        Args:
            task: 任务对象

        """
        if self.is_exists(task):
            self.__store.remove(key=task.task_id)

    def first(self) -> Optional[TaskType]:
        """返回第一个任务(当前任务)

        Returns:
            Task: 第一个任务对象.

        """
        return self.__store.top()

    def finish(self, task: TaskType) -> None:
        """完成并移除任务

        Args:
            task: 任务对象

        """
        task.finish()
        self.remove(task)

    @with_lock
    def terminate(self, error: Union[str, XYZBaseError] = None) -> None:
        """终止管理器集合中的所有任务."""
        if isinstance(error, str):
            error = XYZBaseError(error_message=error)
        for task in self.__store.all():
            task.terminate(error=error)

    def is_exists(self, task: TaskType) -> bool:
        """判断任务是否存在.

        Args:
            task (Task): 任务对象.

        Returns:
            bool: True, 存在; False, 不存在.

        .. versionadded:: 1.4.0
            用于判断任务是否存在.
        """
        return task.task_id in self.__store

    def __contains__(self, task: TaskType) -> bool:
        """使用in来判断是否任务已经存在.

        Args:
            task: 任务对象.

        Returns:
            bool: True 存在, False 不存在.

        """
        return task.task_id in self.__store

    def __len__(self) -> int:
        """返回任务数量.

        Returns:
            int: 任务数量.

        """
        return len(self.__store)

    def __getitem__(self, key: str) -> TaskType:
        """默认下标方式获取任务

        Args:
            key: 下标

        Raises:
            IndexError: 下标越界.

        Returns:
            Task: 任务对象.

        """
        return self.__store[key]

    def get_task_by_index(self, index: int) -> Optional[TaskType]:
        """根据下标获取任务

        Args:
            index: 下标

        Returns:
            Task: Optional. 返回任务对象, 不存在则返回None.

        .. deprecated:: 1.4.0
            禁用通过下标索引获取，使用``get_task_by_id``方法代替.
        """
        warnings.warn(message="禁用通过下标索引获取，使用``get_task_by_id``方法代替.")
        try:
            return self.__store[index] # type: ignore
        except IndexError:
            return None

    def get_task_by_id(self, task_id: str) -> Optional[TaskType]:
        """根据ID获取任务

        Args:
            task_id: 任务ID

        Returns:
            Task: Optional. 返回任务对象, 不存在则返回None.

        """
        return self.__store.get(task_id)

    def get_task_or_404(self, v: Union[int, str]) -> TaskType:
        """获取任务，如果不存在则抛出异常.

        Args:
            v(str or int): 任务ID或者下标号.

        Returns:
            TaskType: 任务对象.

        Raises:
            TypeError: 参数类型错误.
            XYZNotFoundError: 记录不存在
        """
        if isinstance(v, int):
            task = self.get_task_by_index(v)
        elif isinstance(v, str):
            task = self.get_task_by_id(v)
        else:
            raise TypeError("v must be a string or integer.")

        if task is None:
            raise XYZNotFoundError(error_message=f"Task({v}) not found")
        return task

    @with_lock
    def remove_task_by_id(self, task_id: str) -> Optional[TaskType]:
        """通过任务ID移除任务对象.

        Args:
            task_id: 任务ID.

        Returns:
            Task: Optional. 成功移除后返回任务对象, 否则返回None.

        """
        task = self.get_task_by_id(task_id)
        if task is None:
            return
        self.remove(task)
        return task

    @with_lock
    def remove_task_by_index(self, index: int) -> Optional[TaskType]:
        """通过下标移除任务.

        Args:
            index(int): 下标.

        Returns:
            Task: Optional. 成功移除后返回任务对象, 否则返回None.

        .. deprecated:: 1.4.0
            禁用通过下标索引移除，使用``remove_task_by_id``方法代替.
        """
        warnings.warn(message="禁用通过下标索引移除，使用``remove_task_by_id``方法代替.")
        task = self.get_task_by_index(index)
        if task is None:
            return
        self.remove(task)
        return task

    def dumps(self) -> bytes:
        """导出任务.

        Returns:
            bytes: 序列化后的二进制对象.

        """
        data = {}
        for key, val in self.__store.getdict().items():
            data[key] = val.dict()
        return pickle.dumps(data)

    def loads(self, data: bytes):  # type: ignore
        """加载任务.

        Args:
            data: 二进制对象.

        """
        try:
            data: dict = pickle.loads(data)
            if "_TaskManager__tasks_map" in data:
                # 兼容 V1.3.* 之前的版本
                for task_id, task in data.pop(
                    "_TaskManager__tasks_map"
                ).items():
                    self.__store.set(task_id, task)
            elif isinstance(data, MemoryStore):
                # 兼容 V1.4.* 版本
                self.__store = data
            else:
                self.__store = get_store_cls()()
                d = {}
                for task_id, task in data.items():
                    Task = self.__get_orig()
                    d[task_id] = Task(**task)
                self.__store._data = d
        except Exception as e:
            raise XYZBaseError(error_message="Loading Manager Failed") from e

    def clear(self):
        """重置任务."""
        self.__store.clear()

    def __get_orig(self) -> Type[TaskType]:
        """根据泛型参数获取工作空间类.

        Returns:
            Type[T]: 类对象.
        """
        if self.__original_class__ is None:
            try:
                self.__original_class__ = get_args(self.__orig_class__)[0]  # type: ignore
            except IndexError:
                cls_name = self.__class__.__name__
                raise RuntimeError(
                    f"请在 {cls_name} 初始化时指定 {cls_name.replace('Manager', '')}"
                    f" 类. 例如: \nmanager = {cls_name}[{cls_name.replace('Manager', '')}]()"
                ) from None
        return self.__original_class__  # type: ignore


class WorkspaceManager(Generic[WorkspaceType]):
    """工作空间管理类.

    Attributes:
        _workspaces(List[Workspace]): 存储工作空间对象的列表集合.
        _workspaces_map(Dict[str, Workspace]): 存储工作空间对象的字典集合.

    Methods:
        ready(ws_id): 将该ID的工作空间设置为`已就绪`.
        not_ready(ws_id): 将该ID的工作空间设置为`未就绪`.
        get(ws_id): 根据ID返回工作空间对象.
        add(ws): 接收一个工作空间对象, 将其添加至管理器.
        create(ws_id): 传入ID, 创建工作空间对象并添加至管理器.
        create_or_modify(ws_id, is_ready):
            如果ID不存在则创建工作空间并添加.
            如果ID存在, 则更新is_ready属性.
        remove(ws): 移除指定工作空间.
        is_exists(ws): 判断工作空间是否存在, 可传入ws_id或ws.
        is_ready(ws_id): 判断该工作空间是否就绪.
        clear(): 清空当前管理器.
        load(bytes): 加载属性.

    """

    __original_class__: Type[WorkspaceType] = None  # type: ignore
    __sub_instances__ = []

    def __new__(cls, *args, **kwargs):
        if not cls.__sub_instances__:
            cls.__sub_instances__.append(super().__new__(cls))
        return cls.__sub_instances__[0]

    def __init__(self):
        self.__store: Union["MemoryStore", "BaseStore"] = get_store_cls()()

    @classmethod
    def _get_subinstance(cls) -> Optional["TaskManager[WorkspaceType]"]:  # type: ignore
        """获取子类实例.

        Returns:
            WorkspaceManager: 工作空间管理器.
        """
        return cls.__sub_instances__[0] if cls.__sub_instances__ else None

    @property
    def _workspaces(self) -> List[WorkspaceType]:
        return self.__store.all()

    @property
    def _workspaces_map(self) -> Dict[str, WorkspaceType]:
        return self.__store.getdict()

    def ready(self, ws_id: str) -> None:
        """设置工作空间状态为就绪

        Args:
            ws_id(str): 工作空间ID

        Returns:

        """
        ws = self.get_workspace_or_404(ws_id)
        ws.is_ready = True

    def not_ready(self, ws_id: str) -> None:
        """设置工作空间状态未就绪.

        Args:
            ws_id(str): 工作空间ID

        Returns:

        """
        ws = self.get_workspace_or_404(ws_id)
        ws.is_ready = False

    def is_empty(self) -> bool:
        """判断manager是否为空."""
        return len(self.__store) == 0

    def get(self, ws_id: str) -> Optional[WorkspaceType]:
        """根据空间ID获取工作空间对象，不存在时返回 None.

        Args:
            ws_id(str): 工作空间ID.

        Returns:
            workspace(Workspace or None): 工作空间对象

        .. versionchanged:: 1.4.0
            1. 不存在时不会抛出异常，修改为返回 None.
            2. 新增 ``__get_orig`` 方法，用于获取原始的工作空间类.
        """
        return self.__store.get(ws_id)

    def get_workspace_or_404(self, ws_id: str) -> WorkspaceType:
        """根据空间ID获取工作空间对象, 不存在时抛出异常.

        .. versionadded:: 1.4.0
            根据空间ID获取工作空间对象，如果不存在则抛出异常.

        Args:
            ws_id(str): 工作空间ID.

        Returns:
            workspace(Workspace or None): 工作空间对象

        Raises:
            NotFoundWorkspaceError: 未找到该工作空间时, 抛出此异常.
        """
        ws = self.get(ws_id)
        if ws is None:
            raise XYZNotFoundError(
                error_message=f"Workspace({ws_id}) Not Found"
            ) from None
        return ws

    @with_lock
    def add(self, ws: WorkspaceType) -> WorkspaceType:
        """添加工作空间至管理器

        Args:
            ws(Workspace): 工作空间对象.

        Raises:
            WorkspaceExistsError: 工作空间ID已存在时抛出此异常.

        Returns:

        """
        if self.is_exists(ws.ws_id):
            raise XYZIntegrityError(
                error_message=f"Workspace({ws.ws_id}) has already exists"
            )
        self.__store.set(ws.ws_id, ws)
        return ws

    def create(self, ws_id: str, is_ready: bool = True) -> WorkspaceType:
        """创建工作空间并添加至管理器中.

        NOTICE: 当前方法是线程安全的.

        .. versionadded:: 1.4.0
            add parameter `is_ready`.

        Args:
            ws_id(str): 工作空间ID.
            is_ready(bool): 工作空间是否就绪. 默认为True.

        Raises:
            WorkspaceExistsError: 工作空间ID已存在时抛出此异常.

        Returns:
            Workspace: 工作空间对象
        """
        cls = self.__get_orig()
        ws = cls(ws_id=ws_id, is_ready=is_ready)
        return self.add(ws)

    def create_or_modify(self, ws_id: str, is_ready: bool = True) -> WorkspaceType:
        """创建或更新工作空间状态为已就绪.

        NOTICE: 当前方法是线程安全的.

        Args:
            ws_id(str): 工作空间ID.
            is_ready(bool): 是否就绪, 默认已就绪.

        Returns:
            Workspace: 工作空间对象.
        """
        if self.is_exists(ws_id):
            ws = self.get_workspace_or_404(ws_id)
            ws.is_ready = is_ready
        else:
            ws = self.create(ws_id, is_ready=is_ready)
        return ws

    append = create

    @with_lock
    def remove(self, ws: WorkspaceType) -> None:
        """移除指定的工作空间.

        Args:
            ws(Workspace): 工作空间对象

        Returns:

        """
        self.__store.remove(ws.ws_id)

    def is_exists(self, item: Union[str, WorkspaceType]) -> bool:
        """判断工作空间是否已存在

        Args:
            item: 工作空间ID或工作空间对象

        Raises:
            TypeError: item仅为str或Workspace类型.

        Returns:

        """
        if isinstance(item, str):
            ws_id = item
        elif hasattr(item, "ws_id"):
            ws_id = item.ws_id
        else:
            raise TypeError
        return ws_id in self.__store

    def is_ready(self, ws_id: str) -> bool:
        """根据空间ID判断, 当前空间是否已就绪.

        Args:
            ws_id(str): 工作空吗ID.

        Returns:

        """
        ws = self.get_workspace_or_404(ws_id)
        return ws.is_ready

    def __contains__(self, item: Union[str, WorkspaceType]) -> bool:
        """判断工作空间是否已存在

        Args:
            item: 工作空间ID或工作空间对象

        Raises:
            TypeError: item仅为str或Workspace类型.

        Returns:

        """
        return self.is_exists(item)

    def dumps(self) -> bytes:
        """导出任务"""
        return pickle.dumps(self.__store)

    def loads(self, data: bytes):
        """加载任务.

        Args:
            data: 二进制对象.

        """
        try:
            obj = pickle.loads(data)
            old_dict = obj.__dict__
            # 兼容以前的数据，如果是以前的数据，则需要把以前的数据转换成新的数据格式
            if "_workspaces_map" in old_dict:
                for ws_id, ws in old_dict.pop("_workspaces_map").items():
                    self.__store.set(ws_id, ws)
            elif isinstance(obj, BaseStore):
                self.__store = obj
        except Exception as e:
            raise XYZBaseError(error_message="Loading Manager Failed") from e

    def clear(self):
        """清空管理器."""
        self.__store.clear()

    def __get_orig(self) -> Type[WorkspaceType]:
        """根据泛型参数获取工作空间类.

        Returns:
            Type[T]: 类对象.
        """
        if self.__original_class__ is None:
            try:
                self.__original_class__ = get_args(self.__orig_class__)[0]  # type: ignore
            except IndexError:
                cls_name = self.__class__.__name__
                raise RuntimeError(
                    f"请在 {cls_name} 初始化时指定 {cls_name.replace('Manager', '')}"
                    f" 类. 例如: \nmanager = {cls_name}[{cls_name.replace('Manager', '')}]()"
                ) from None
        return self.__original_class__  # type: ignore
