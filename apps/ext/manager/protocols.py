"""
定义Order、Task、Workspace需要遵循的最基本协议
"""
from typing import Protocol, TYPE_CHECKING

from pydantic import BaseModel

if TYPE_CHECKING:
    from apps.exceptions import XYZBaseError


class OrderProto(Protocol):
    """
    订单协议类
    """
    _order: BaseModel

    def __init__(self, order_id: str, *arg, **kwargs):
        pass

    @property
    def order_id(self) -> str:
        pass

    def finish(self):
        pass

    def terminate(self, error: "XYZBaseError", is_remove: bool = True):
        pass

    def dict(self, *args, **kwargs) -> dict:
        pass

    def json(self, *args, **kwargs) -> str:
        pass


class TaskProto(Protocol):
    """
    任务协议类
    """
    _task: BaseModel

    def __init__(self, task_id: str, *args, **kwargs):
        pass

    @property
    def task_id(self) -> str:
        pass

    def finish(self):
        pass

    def terminate(self, error: "XYZBaseError", is_remove: bool = True):
        pass

    def dict(self, *args, **kwargs) -> dict:
        pass

    def json(self, *args, **kwargs) -> str:
        pass


class WorkspaceProto(Protocol):
    """工作空间协议类"""
    ws_id: str
    is_ready: bool

    def __init__(self, ws_id: str, is_ready: bool = True, *args, **kwargs):
        pass

    def dict(self, *args, **kwargs) -> dict:
        pass

    def json(self, *args, **kwargs) -> str:
        pass
