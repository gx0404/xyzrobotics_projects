#!/usr/bin/env python
'''
Copyright (c) XYZ Robotics Inc. - All Rights Reserved
Unauthorized copying of this file, via any medium is strictly prohibited
Proprietary and confidential
Author: Xiao Ao <ao.xiao@xyzrobotics.ai>, January, 2021
'''
import os
import subprocess

from flask import request, current_app
from flask_restful import Resource

from apps.settings import settings
from apps.globals import mp, hub_client, openapi
from apps.base_app.flask_hook import big_event, req_log
from apps.base_app.views.validations import StartNodesSchema
from apps.helpers import make_json_response
from apps.std_log_msg import pause_success, start_success, stop_success
from apps.log import hmi_log
from apps.utils import node
from xyz_central_hub.client import HubClient

STATE_MACHINE_PATH = settings.STATE_MACHINE_PATH
RAFCON_DEBUG = settings.RAFCON_DEBUG

if not RAFCON_DEBUG:
    from xtf_ros.client import start, stop, pause, switch_flow


# TODO uniform start stop pause go_on with hmi


class Start(Resource):
    @req_log(ignoreReq=True)
    @big_event("START")
    @openapi.api_doc(tags=["XLHB(标准库接口汇总)", "操作命令"], summary="启动系统")
    def get(self):
        if not RAFCON_DEBUG:
            switch_flow(str(STATE_MACHINE_PATH[0]))
            start()
        mp.system(start_success)
        current_app.status = "ready"
        ret = make_json_response()
        return ret


class Pause(Resource):
    @req_log(ignoreReq=True)
    @big_event("PAUSE")
    @openapi.api_doc(tags=["XLHB(标准库接口汇总)", "操作命令"], summary="暂停系统")
    def get(self):
        stop_xtf()
        ret = make_json_response()
        return ret

def stop_xtf():
    client = HubClient()
    if not RAFCON_DEBUG:
        try:
            stop()
        except Exception as e:
            hmi_log.warn(e)
            hmi_log.warn("Fail to stop xtf with api! Stop the xtf node instead")
            client.stop_node("xtf")
    mp.system(stop_success)
    current_app.status = "stopped"

class Stop(Resource):

    @req_log(ignoreReq=True)
    @big_event("Stop")
    @openapi.api_doc(tags=["XLHB(标准库接口汇总)", "操作命令"], summary="停止系统")
    def get(self):
        stop_xtf()
        node.stop_robot_node()
        ret = make_json_response()
        return ret


class StartNode(Resource):
    def __init__(self):
        # self.schema = NodeIdSchema()
        self.schema = StartNodesSchema()

    @req_log()
    @openapi.api_doc(tags=["XLHB(标准库接口汇总)", "操作命令"], summary="开启一个节点", request_body=StartNodesSchema)
    def post(self):
        """Start single node."""
        error = None
        data, error = self.schema.loads(request.data)
        if error:
            print(error)
            return make_json_response(code=-1, msg="data type error")
        else:
            res = hub_client.start_node(data['node_id'], restart=True)
            return make_json_response(data=res)


class RestartNode(Resource):
    def __init__(self):
        self.schema = StartNodesSchema()

    @req_log()
    @openapi.api_doc(tags=["XLHB(标准库接口汇总)", "操作命令"], summary="重启一个节点", request_body=StartNodesSchema)
    def post(self):
        """Start single node."""
        error = None
        data, error = self.schema.loads(request.data)

        if error:
            print(error)
            return make_json_response(code=-1, msg="data type error")
        else:
            node_id = data['node_id']
            nodes_status = hub_client.get_node_info()
            if nodes_status.get("error_code", -1) < 0:
                msg = nodes_status.get("error_msg", "central hub client error")
                return make_json_response(code=-1, msg=msg)

            node_list = nodes_status.get("node_list")
            if not node_list:
                msg = "node list is empty"
                return make_json_response(code=-1, msg=msg)

            # TODO(Yuhang Wu): 待优化, 判断接受的node_id是否存在, 不存在则抛出异常
            for node_msg in nodes_status["node_list"]:
                if node_msg["node_id"] == node_id:
                    break
            else:
                msg = "node :%s is not found" % node_id
                return make_json_response(code=-1, msg=msg)

            # node_state = node_msg.get("node_state", False)
            res = hub_client.start_node(node_id, restart=True)
            return make_json_response(data=res)


class Shutdown(Resource):
    @req_log(ignoreReq=True)
    @openapi.api_doc(tags=["XLHB(标准库接口汇总)", "操作命令"], summary="关闭操作系统", deprecated=True)
    def get(self):
        os.system('systemctl poweroff -i')
        std_res = make_json_response()
        return std_res


class ResetCamera(Resource):
    def __init__(self, **kwargs):
        self.ws_node = kwargs['ws_node']

    @req_log(ignoreReq=True)
    @openapi.api_doc(tags=["XLHB(标准库接口汇总)", "操作命令"], summary="关闭相机", deprecated=True)
    def get(self):

        hub_client.stop_node('active_vision')
        hub_client.stop_node('passive_vision')
        hub_client.stop_node('place_vision')
        res = subprocess.call(
            "echo robot2018sh | sudo -S bash /usr/local/bin/rebind_camera.sh", shell=True)
        self.ws_node.start_background_task(self.restart_vision_node)
        if res != 0:
            std_res = make_json_response(code=-1)
            return std_res
        else:
            std_res = make_json_response()
        return std_res

    def restart_vision_node(self):
        active_start_res = hub_client.start_node('active_vision')
        passive_start_res = hub_client.start_node('passive_vision')
        place_start_vision = hub_client.start_node('place_vision')
        mp.vision.active(active_start_res)
        mp.vision.passive(passive_start_res)
        mp.vision.place(place_start_vision)

class UpdatePlcSignal(Resource):
    @req_log()
    @openapi.api_doc(tags=["XLHB(标准库接口汇总)", "操作命令"], summary="查询三色灯", deprecated=True)
    def post(self):
        """返回三色灯状态
        """
        data = {
            "lights": current_app.plc_light,
            "green": current_app.plc_light[0],
            "yellow": current_app.plc_light[1],
            "red": current_app.plc_light[2]
        }
        return make_json_response(data=data)

class GetStatus(Resource):
    @req_log()
    @openapi.api_doc(tags=["XLHB(标准库接口汇总)", "操作命令"], summary="获取系统状态")
    def get(self):
        """
        Returns:
            ---
            {"code": 0, "message": "success", "data": {"status": "ready"}}
            ---
        """
        ret = make_json_response(data={"status": current_app.status})
        return ret

class GetMode(Resource):
    @req_log(ignoreReq=True)
    @openapi.api_doc(tags=["XLHB(标准库接口汇总)", "操作命令"], summary="获取系统模式")
    def get(self):
        """
        Returns:
            ---
            {"code": 0, "message": "success", "data": {"status": "robot"}}
            ---
        """
        ret = make_json_response(data={"status": current_app.mode})
        return ret
