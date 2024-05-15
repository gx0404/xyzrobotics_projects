# -*- coding: utf-8 -*-
'''
Copyright (c) XYZ Robotics Inc. - All Rights Reserved
Unauthorized copying of this file, via any medium is strictly prohibited
Proprietary and confidential
Author: Michael.Su <michael.su@xyzrobotics.ai>, April, 2022
'''
from io import BytesIO
from flask_restful import Resource
from flask import request
from apps.base_app.flask_hook import req_log
from apps.enums import MediaType
from apps.globals import openapi
from apps.helpers import make_json_response
from apps.base_app.views.validations import DownloadNodeLogSchema, NodeLogSchema
from apps.log import hmi_log
from apps import settings
from xmlrpc.client import ServerProxy
from threading import Thread

from apps.responses import StreamingResponse
from apps.utils.dt import now_timestamp
from .read_node_log import node_log_reader_thread, NodeLogReader
from zipfile import ZipFile
from flask_cors import cross_origin
import os

class ProcessUtils():
    def __init__(self):
        self.supervisor_client = ServerProxy('http://127.0.0.1:{}'.format(settings.SUPERVISOR_PORT))
        self.process_stdout_thread = {}

    def get_process_path(self, node_name):
        """supervisor.getAllProcessInfo()
        Args:
            node_name (String)
        
        Return:
            programm path (string)
        
        Raise:
            KeyError
        """
        for info in self.supervisor_client.supervisor.getAllProcessInfo():
            if info["name"] == node_name:
                return info["logfile"]
        
        raise KeyError("{} is not found in supervisor processes".format(node_name))

    def read_process_log(self, node_name, lines=500):
        """read the latest process log
        Args:
            node_name (String)
            lines (Int). Default: 500
        Return:
            log_list
            log_exist (Bool) : True/False
        Raise:
            KeyError (node_name)
        """  
        file_path = self.get_process_path(node_name)
        if not os.path.exists(file_path):
            hmi_log.warn("{} doesn't exist!".format(file_path))
            return [], False
        with open(file_path, "r") as fin:
            data = fin.readlines()
            file_line = len(data)
            if file_line >= lines:
                return data[file_line-lines:], True
            else:
                return data, True  

    def create_process_stdout_thread(self, node_name, requesting):
        """
        Args:
            node_name (String)
        
        Raise:
            KeyError (node_name)
        """
        def new_thread() -> Thread:
            return Thread(
                target=node_log_reader_thread,
                args=(
                    self.process_stdout_thread[node_name]["node_log_reader"],
                    node_name,
                    requesting,
                    lambda: self.process_stdout_thread[node_name]["stop_signal"]
                ),
                daemon=True
            )

        if node_name not in self.process_stdout_thread:
            self.process_stdout_thread[node_name] = {"stop_signal": False}
            node_log_reader = NodeLogReader(self.get_process_path(node_name))
            self.process_stdout_thread[node_name]["node_log_reader"] = node_log_reader
            t = new_thread()
            self.process_stdout_thread[node_name]["thread"] = t
            t.start()
        else:
            is_alive = self.process_stdout_thread[node_name]["thread"].is_alive()
            if not is_alive:
                t = new_thread()
                del self.process_stdout_thread[node_name]["thread"]
                self.process_stdout_thread[node_name]["thread"] = t
                t.start()


process_utils = ProcessUtils()




class NodeLog(Resource):
    def __init__(self, *args, **kwargs):
        from apps.utils.requesting import requesting

        self.schema = NodeLogSchema()
        # TODO: self.requesting 可以删除，没有使用到
        self.requesting = requesting

    @req_log(ignoreRes=True)
    @openapi.api_doc(tags=["XLHB(标准库接口汇总)", "日志"], summary="查询节点日志", request_body=NodeLogSchema)
    def post(self):
        """ Get n logs and create thread to monitor log file change and publish changes by websocket

        Body:
            ---
            {
                "node_name": "0-robot_node",
                "num_of_log": 1000
            }
            ---

        Returns:
            ---
            {
            "code": 0,     
            "message": "ok",
            "log":[ 
                    "XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX",
                    "XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX",
                    "XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX"        
                ]
            }
            ---
        """
        # Validate input data
        msg, error = self.schema.load(request.get_json())
        if error:
            hmi_log.error("{} {}".format(msg, error))
            res = make_json_response(code=-1, msg="format error")
            return res

        # Get node log path
        log, log_exist = process_utils.read_process_log(msg["node_name"], lines=msg["num_of_log"])
        if log_exist:
            process_utils.create_process_stdout_thread(msg["node_name"], self.requesting)
        else:
            hmi_log.warn("Will not create thread to monitor {} log".format(msg["node_name"]))
        return {
            "code": 0,
            "message": "Success",
            "log": log
        }

class DownloadNodeLog(Resource):
    def __init__(self):
        self.schema = DownloadNodeLogSchema()

    @cross_origin(expose_headers=["filename"])
    @openapi.api_doc(tags=["XLHB(标准库接口汇总)", "日志"], summary="下载节点日志", request_body=DownloadNodeLogSchema)
    def post(self):
        """ Download node log. 

        There could be multiple logs. Use zip 

        Args:
            {
                "node_name": "4-robot_node",
            }   

        Return
            zip_data,
            mimetype='zip',
            as_attachment=True,
            attachment_filename="{}.zip".format(os.path.splitext(log_filename))

        """
        # Validate input data
        msg, error = self.schema.load(request.get_json())
        if error:
            hmi_log.error("{} {}".format(msg, error))
            res = make_json_response(code=-1, msg="format error")
            return res

        # Get node log path
        node_log_path = process_utils.get_process_path(msg["node_name"])
        if not os.path.exists(node_log_path):
            msg = "[] does not exist".format(node_log_path)
            res = make_json_response(code=-1, msg=msg)
            return res
        
        # create zip file
        log_dir_name, log_filename = os.path.split(node_log_path)
        log_zip_file_apth = os.path.join(log_dir_name, os.path.splitext(log_filename)[0] + ".zip")
        if os.path.exists(log_zip_file_apth):
            os.remove(log_zip_file_apth)

        zip_obj = ZipFile(log_zip_file_apth, 'w')
        for log_file in os.listdir(log_dir_name):
            if log_filename in log_file:
                hmi_log.info(f"打包文件: {log_file}")
                zip_obj.write(os.path.join(log_dir_name, log_file), log_file)
        zip_obj.close()

        with open(log_zip_file_apth, 'rb') as f:
            zip_data = BytesIO(f.read())

        # remove zip file
        os.remove(log_zip_file_apth)

        return StreamingResponse(
            filename=f"{msg['node_name']}-{now_timestamp()}.zip",
            stream=zip_data,
            media_type=MediaType.zip,
            as_attachment=True
        )

