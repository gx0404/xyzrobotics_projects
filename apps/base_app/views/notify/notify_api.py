#!/usr/bin/env python
'''
Copyright (c) XYZ Robotics Inc. - All Rights Reserved
Unauthorized copying of this file, via any medium is strictly prohibited
Proprietary and confidential
Author: Xiao Ao <ao.xiao@xyzrobotics.ai>, January, 2021
'''
import json

from flask import request
from flask_restful import Resource

from apps.globals import mp, openapi
from apps.base_app.flask_hook import req_log
from apps.base_app.views.validations import NodeErrSchema, SysLogSchema
from apps.helpers import make_json_response
from apps.utils.dt import now_timestamp


class NotifyNodeErr(Resource):
    def __init__(self):
        self.schema = NodeErrSchema()

    @req_log()
    @openapi.api_doc(tags=["XLHB(标准库接口汇总)", "消息推送"], summary="通知节点异常", request_body=NodeErrSchema)
    def post(self):

        msg, error = self.schema.loads(request.data)
        if error:
            print(error)
            res = make_json_response(code=-1, msg="format error")
            return res

        try:
            mp.node_error_log(msg)
            std_res = make_json_response()
        except Exception:
            std_res = make_json_response(code=-1)
        return std_res


class NotifySystemLog(Resource):
    def __init__(self, **kwargs):
        self.schema = SysLogSchema()

    @req_log()
    @openapi.api_doc(tags=["XLHB(标准库接口汇总)", "消息推送"], summary="通知系统异常", request_body=SysLogSchema)
    def post(self):

        data, error = self.schema.loads(request.data)
        if error:
            res = make_json_response(code=-1, msg='data type error')
            return res
        data = json.loads(self.schema.dumps(data)[0])  # stupid but simple

        timestamp = now_timestamp()
        data['timestamp'] = timestamp
        try:
            mp.system(data)
            return make_json_response()
        except Exception:
            return make_json_response(code=-1)


class TriggleWSEvent(Resource):

    @openapi.api_doc(tags=["XLHB(标准库接口汇总)", "消息推送"], summary="自定义消息推送")
    def post(self):
        """
        Body:
            ---
            {"event": "system_log", "data": ""}
            ---
        """
        data = request.get_json()
        assert "event" in data
        assert "data" in data
        mp.push(data["event"], data["data"])
        return make_json_response()


class PublishOrderLog(Resource):

    @req_log()
    @openapi.api_doc(tags=["XLHB(标准库接口汇总)", "消息推送"], summary="订单消息推送")
    def post(self):
        """
        Body:
            ---
            {"message": "xxx", "status": True}
            ---
        """
        data = request.get_json()
        mp.order(data["message"], data["status"])
        return make_json_response()
