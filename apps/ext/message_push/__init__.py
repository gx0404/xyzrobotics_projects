"""
统一的消息推送接口

示例:
    from apps import mp

    # 推送消息
    mp.send(event="xxx", data={"xxx": "xxx"})

    # 使用事件属性推送消息
    mp.order.info(data={"xxx": "xxx"})
    mp.system.error(error_message={})
"""
from .push import MessagePush

__all__ = ["MessagePush"]
