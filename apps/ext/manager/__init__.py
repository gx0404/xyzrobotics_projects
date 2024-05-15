"""
manager模块包括：TaskManager, OrderManager, WorkspaceManager
它们分别用于任务、订单、工作空间的管理

protocols.py
    定义了任务、订单与工作空间最基本的协议（接口），例如子任务开发的任务类必须具备协议中定义的基本属性与方法
"""
from .manager import TaskManager, OrderManager, WorkspaceManager, with_lock
from .protocols import TaskProto, OrderProto, WorkspaceProto


__all__ = [
    "TaskManager",
    "OrderManager",
    "WorkspaceManager",
    "TaskProto",
    "OrderProto",
    "WorkspaceProto",
    "with_lock",
]
