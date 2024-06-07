"""
光栅控制接口

当有任务正在执行时，不能对光栅进行操作，仅有任务的情况才可以对光栅进行操作。

系统就绪状态, 操作光栅信号将影响到 XTF
"""
import typing as t

from apps import wcs_log
from apps import SimpleBlueprint
from apps.helpers import make_json_response
from apps.utils import node, plc
from flask import current_app  # type: ignore

from wcs_adaptor.manager import task_manager
from wcs_adaptor.exceptions import InvalidOperationError, WCSError

if t.TYPE_CHECKING:
    from apps import Application


current_app: "Application"
sbp = SimpleBlueprint("grating", __name__, url_prefix="/api/grating", logger=wcs_log)


@sbp.post("/disable")
def disable():
    """禁用光栅"""
    if task_manager.first():
        raise InvalidOperationError("任务进行中，无法禁用光栅")
    try:
        plc.disable_grating()
    except Exception as exc:
        raise WCSError("光栅禁用失败") from exc
    if current_app.status == "ready":
        node.stop_xtf(current_app)
    return make_json_response(msg="光栅已禁用")


@sbp.post("/enable")
def enable():
    """启用光栅"""
    if task_manager.first():
        raise InvalidOperationError("任务进行中，无法开启光栅")
    try:
        plc.enable_grating()
    except Exception as exc:
        raise WCSError("光栅启动失败") from exc
    if current_app.status == "ready":
        node.start_xtf(current_app)
    return make_json_response(msg="光栅已开启")
