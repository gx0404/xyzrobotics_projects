#!/usr/bin/python
# -*- coding: utf-8 -*-
'''
Copyright (c) XYZ Robotics Inc. - All Rights Reserved
Unauthorized copying of this file, via any medium is strictly prohibited
Proprietary and confidential
'''

import time
import json
import socket
from central_hub_config import ConfigManager


class NodeClient(object):
    """
    """
    def __init__(self, tcp_addr=None):
        """
        Args:
            tcp_addr(tuple): tcp address such as ("127.0.0.1", 65535),
                if tcp_addr is none will get from config
        """
        self._node_client_socket = None
        self._tcp_addr = tcp_addr
        self._init_tcp_addr()
        super(NodeClient, self).__init__()

    def __del__(self):
        """
        """
        if self._node_client_socket is not None:
            self._node_client_socket.close()

    def _init_tcp_addr(self):
        """ init tcp address 
        """
        if self._tcp_addr is None:
            config_manager = ConfigManager()
            self._tcp_addr = config_manager.get_socket_addr()

    def _init_socket(self):
        """ create client socket
        """
        self._node_client_socket = socket.socket(socket.AF_INET,
                                                 socket.SOCK_STREAM)
        self._node_client_socket.connect(self._tcp_addr)

    def send_msg(self, msg):
        """ send msg to node server
        Args:
            msg(str): msg send to node server shoud be a json string
            For example:
            data =  {"node_name": "node name",
                     "timestamp": 123456789, //unix timestamp unit: ms 
                     "error_code": 0, // 0 means no error
                     "error_msg": "msg"}
            msg = json.dumps(data)
        """
        self._init_socket()
        size = len(msg)
        header = "%10d000" % len(msg)
        data = msg
        self._node_client_socket.send(header.encode("utf-8"))
        self._node_client_socket.sendall(data.encode("utf-8"))
        self._node_client_socket.close()


if __name__ == "__main__":
    """
    """ 
    msg = {}
    msg["node_name"] = "test6"
    msg["timestamp"] = int(1000 * time.time())
    msg["error_code"] = 0
    msg["error_msg"] = "testing"
    node_client = NodeClient()
    node_client.send_msg(json.dumps(msg))
    while True:
        time.sleep(60)
