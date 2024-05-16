# -*- coding: utf-8 -*-
"""
Copyright (c) XYZ Robotics Inc. - All Rights Reserved
Unauthorized copying of this file, via any medium is strictly prohibited
Proprietary and confidential
Author: Yuhang Wu <yuhang.wu@xyzrobotics.ai>, 2022-04-12
"""
import json
import unittest

import requests
import jsonschema
# from xyz_central_hub.client import HubClient


class TestCommand(unittest.TestCase):
    """
    Test command api.
    """

    def setUp(self) -> None:
        self.client = requests.session()
        # self.hub_client = HubClient()
        self.host = "http://localhost:7002"

    def test_start_rafcon(self):
        """Test start rafcon"""

        url = "/api/cmd/start"
        resp_raw = self.client.get(self.host + url)
        resp = resp_raw.json()
        # 成功示例
        success_data = {"code": 0, "msg": "success", "data": {}}
        assert success_data == resp

    def test_stop_rafcon(self):
        """Test stop rafcon"""

        url = "/api/cmd/stop"
        resp_raw = self.client.get(self.host + url)
        resp = resp_raw.json()
        # 成功示例
        success_data = {"code": 0, "msg": "success", "data": {}}
        assert resp == success_data

    def test_pause_rafcon(self):
        """Test pause rafcon"""

        url = "/api/cmd/pause"
        resp_raw = self.client.get(self.host + url)
        resp = resp_raw.json()
        success_data = {"code": 0, "msg": "success", "data": {}}
        assert resp == success_data

    def test_update_plc_signal(self):
        """Test update plc signal api
        
        The API responsed successfully:
            {
                "code": 0,
                "msg": "success",
                "data": {
                    "lights": [0,0,0],
                    "green": 0,
                    "yellow": 0,
                    "red": 0
                }
            }
        """
        url = "/api/cmd/update_plc_signal"
        resp_raw = self.client.post(self.host + url)
        resp = resp_raw.json()
        assert resp["code"] == 0, resp
        assert resp["data"]["red"] == 0, resp

    # def test_start_node(self):
    #     """Test start a node."""
    #
    #     url = "/api/cmd/start_node"
    #     body = {"node_id": "xvf_no_gui"}
    #     resp_raw = self.client.post(self.host + url, json=body)
    #     resp = resp_raw.json()
    #
    #     # read schema file.
    #     with open("./schema/command_node.json") as f:
    #         text = f.read()
    #         schema = json.loads(text)
    #
    #     # validate resp.
    #     jsonschema.validate(resp, schema)
    #
    #     # 为不影响后续测试, 主动停止 node.
    #     self.stop_node(node_id=body["node_id"])
    #
    # def test_restart_node(self):
    #     """Test restart a node."""
    #
    #     url = "/api/cmd/restart_node"
    #     body = {"node_id": "xvf_no_gui"}
    #     resp_raw = self.client.post(self.host + url, json=body)
    #     resp = resp_raw.json()
    #
    #     with open("./schema/command_node.json") as f:
    #         text = f.read()
    #         schema = json.loads(text)
    #     jsonschema.validate(resp, schema)
    #     self.stop_node(node_id=body["node_id"])
    #
    # def test_reset_camera(self):
    #     """Test rest camera."""
    #
    #     url = "/api/cmd/reset_webcam"
    #     resp_raw = self.client.get(url)
    #     resp = json.loads(resp_raw.data.decode("utf-8"))
    #
    #     success = {"code": 0, "msg": "success", "data": {}}
    #     assert resp == success

    # def test_shutdown(self):
    #     """Test shutdown system."""
    #
    #     url = "/api/cmd/shutdown"
    #     resp_raw = self.client.get(url)
    #     resp = json.loads(resp_raw.data.decode("utf-8"))
    #
    #     success = {"code": 0, "msg": "success", "data": {}}
    #     assert success == resp

    # def stop_node(self, node_id):
    #     """Stop a node.
    #
    #     Args:
    #         node_id(str): NodeID.
    #
    #     Returns:
    #
    #     """
    #     self.hub_client.stop_node(node_id)
