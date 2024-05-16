import typing as t
from typing import Optional

from apps import cached_app, settings

if t.TYPE_CHECKING:
    from apps import Application


def stop_robot_node(
    app: Optional["Application"] = None, force_stop: bool = False
) -> None:
    """停用机器人节点

    - 当 STOP_ROBOT_IF_ERROR 配置为 True 时，将会停用机器人节点
    - 当 STOP_ROBOT_IF_STOP 配置为 True 时，将会停用机器人节点

    Args:
        app(Application):
        force_stop(bool): 默认 False，是否强制停止，忽略其他配置，当设置为 True 时，一旦调用此方法将停止机器人节点
    """
    if app is None:
        app = cached_app()

    if app.hub_client is None:
        from xyz_central_hub.client import HubClient

        hub_client = HubClient()
    else:
        hub_client = app.hub_client

    if force_stop:
        hub_client.stop_node("robot_node")
    elif settings and settings.STOP_ROBOT_IF_ERROR or settings.STOP_ROBOT_IF_STOP:
        hub_client.stop_node("robot_node")


def stop_xtf(app: Optional["Application"] = None):
    """停用 XTF 节点"""
    if app is None:
        app = cached_app()

    if app.hub_client is None:
        from xyz_central_hub.client import HubClient

        hub_client = HubClient()
    else:
        hub_client = app.hub_client
    hub_client.stop_node("xtf")


def start_xtf(app: Optional["Application"] = None):
    """开启 XTF 节点"""
    if app is None:
        app = cached_app()

    if app.hub_client is None:
        from xyz_central_hub.client import HubClient

        hub_client = HubClient()
    else:
        hub_client = app.hub_client
    hub_client.start_node("xtf")
