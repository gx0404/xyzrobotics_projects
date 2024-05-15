#!/usr/bin/env python
'''
Copyright (c) XYZ Robotics Inc. - All Rights Reserved
Unauthorized copying of this file, via any medium is strictly prohibited
Proprietary and confidential
Author: Xiao Ao <ao.xiao@xyzrobotics.ai>, January, 2021
'''
import warnings

import requests

from apps.enums import RobotStatus
from .prepare import save_log
from apps.log import outside_log as log
from apps.helpers import json_dumper

SYSTEM_LOG_EVENT = "system_log"
NODE_ERROR_EVENT = "node_error"
# TODO: 使用的大驼峰命名法，之后需要前后端统一修改为下划线命名法
ROBOT_STATUS_EVENT = "RobotStatus"


# Deprecated
class Requesting(object):
    """
    As HMI Bridge
    """

    def __init__(self, socketio):
        self.socketio = socketio
        self.__deprecated_warning = False

    def send_log_message(self, data):
        """Send some messages to outsides, such as HMI and WCS.

        Args:
            msg: A dict
        Returns:
            A bool
        """
        save_log(data)
        return self.send_msg_by_websocket(SYSTEM_LOG_EVENT, data)

    def send_msg_by_websocket(self, event, data):
        if not self.__deprecated_warning:
            warnings.warn(
                "`requesting` 废弃警告，请使用 `mp` 代替，`mp` 使用文档：https://161.189.84.82:8003/xyz-release-doc/ubuntu2004/hmi/latest/tutorial/message-sending/",
                DeprecationWarning,
            )
            self.__deprecated_warning = True

        try:
            self.socketio.emit(event, data)
            serial = json_dumper(data)
            if len(serial) > 100:
                serial = serial[:101]+" ..."
            # log.info(u"[WebSocket] %s: %s" % (event, serial))
            return True
        except Exception:
            log.error(u"[WebSocket] error event: %s" % event, exc_info=True)
            return False

    def send_msg_by_http(self, url, data):
        res = requests.post(url=url, json=data)
        res.raise_for_status()
        return res.json()

    def send_drop_msg(self, source_tote_code, target_tote_code, barcode, num):
        # TODO
        pass

    def send_batch(self, batch_dict):
        # TODO
        pass

    def send_node_error(self, data):
        return self.send_msg_by_websocket(NODE_ERROR_EVENT, data)
    
    def send_robot_status(self, status: RobotStatus) -> None:
        """推送机器人状态给websocket客户端"""
        self.send_msg_by_websocket(ROBOT_STATUS_EVENT, { "status": status.value })
