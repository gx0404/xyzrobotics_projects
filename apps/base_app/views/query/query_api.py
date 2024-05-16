#!/usr/bin/env python
'''
Copyright (c) XYZ Robotics Inc. - All Rights Reserved
Unauthorized copying of this file, via any medium is strictly prohibited
Proprietary and confidential
Author: Xiao Ao <ao.xiao@xyzrobotics.ai>, January, 2021
'''
import csv
import datetime
import io
import json
import os
import sys
import zipfile

from flask import make_response, request
from flask_restful import Resource
from sqlalchemy import or_

from apps.globals import hub_client, openapi
from apps.settings import settings
from apps.base_app.flask_hook import req_log
from apps.base_app.views.validations import (
    BackLogSchema,
    QueryLogMenuSchema,
    QueryLogSchema
)
from apps.helpers import make_json_response
from apps.models import Log
from apps.utils.dt import to_timestamp_from_str as trans_timestamp
from apps.utils.util import get_robot_status

CMCT_LOG_PATH = settings.CMCT_LOG_PATH


class QueryAlive(Resource):
    """No need to log"""
    @openapi.api_doc(tags=["XLHB(标准库接口汇总)", "查询"], summary="心跳接口")
    def get(self):
        return {"echo": "alive"}


class QueryNodesInfo(Resource):

    @req_log()
    @openapi.api_doc(tags=["XLHB(标准库接口汇总)", "查询"], summary="查询所有节点")
    def get(self):
        """Gain all nodes info.
        
        Returns:
            ---
            {
                "code": 0,
                "msg": "success",
                "data": {
                    "node_list": [
                        {
                            "node_id": "robot_config",
                            "node_name": "Robot Configuration",
                            "node_state": True,
                            "program_name": "1-robot_config",
                            "monitor": False
                        }
                    ],
                    "error_msg": None,
                    "error_code": 0
                }
            }
            ---
        """
        res = hub_client.get_node_info()
        if res.get("error_code", -1) >= 0:
            return make_json_response(data=res)
        else:
            return make_json_response(code=-1)


class QueryLog(Resource):
    def __init__(self):
        self.schema = QueryLogSchema()
        
    @req_log(ignoreRes=True)
    @openapi.api_doc(tags=["XLHB(标准库接口汇总)", "日志"], summary="查询节点日志", request_body=QueryLogSchema)
    def post(self):
        error = None
        data, error = self.schema.loads(request.data)

        if error:
            print(error)
            msg = 'data type error'
            res = make_json_response(code=-1, msg=msg)
            return res
        print("!!!",self.schema.dumps(data))
        data = json.loads(self.schema.dumps(data)[0])  # stupid but simple
        req_data = self.get_data_from_db(data)
        return make_json_response(data=req_data)

    @staticmethod
    def get_data_from_db(data):
        star_timestamp = trans_timestamp(data['date'])
        star_date = datetime.datetime.strptime(data['date'], "%Y_%m_%d")
        end_date = (star_date + datetime.timedelta(days=1)).strftime('%Y_%m_%d')
        end_timestamp = trans_timestamp(str(end_date))

        where_clauses = []
        where_clauses.append(Log.timestamp.between(star_timestamp, end_timestamp))

        or_args = [Log.msg_type == i for i in data["filters"]["msg_type"]]
        where_clauses.append(or_(*or_args))

        or_args = [Log.class_ == i for i in data["filters"]["class"]]
        where_clauses.append(or_(*or_args))

        or_args = [Log.tag == i for i in data["filters"]["tag"]]
        where_clauses.append(or_(*or_args))

        if where_clauses:
            res = Log.query.filter(*where_clauses).order_by(-Log.timestamp).all()
        else:
            res = Log.query.order_by(-Log.timestamp).all()

        error_logs, warning_logs, info_logs = [], [], []
        for log in res:
            log_dict = log.to_dict()
            if log_dict["msg_type"] == "error":
                error_logs.append(log_dict)
            elif log_dict["msg_type"] == "warning":
                warning_logs.append(log_dict)
            elif log_dict["msg_type"] == "info":
                info_logs.append(log_dict)

        return {'errors': error_logs, 'warns': warning_logs, 'infos': info_logs}


class DownloadLogs(Resource):
    # TODO: 需要支持响应是文件的情况
    @req_log(ignoreReq=True, ignoreRes=True)
    def get(self):

        logs = [log.to_dict() for log in Log.query.all()]
        for log in logs:
            time = datetime.datetime.fromtimestamp(
                log['timestamp']).strftime('%Y-%m-%d %H:%M:%S')
            log['timestamp'] = time
            for k in log:
                log[k] = log[k].encode("utf-8") if hasattr(log[k],"encode") else log[k]

        csv_data = ""
        if len(logs) != 0:
            headers = logs[0].keys()
            # io
            if sys.version_info.major == 2:
                output = io.BytesIO()
            else:
                output = io.StringIO()
            # csv writer
            writer = csv.DictWriter(output, headers)
            writer.writeheader()
            writer.writerows(logs)
            # read from io
            csv_data = output.getvalue()
        response = make_response(csv_data)
        response.headers['content-type'] = 'application/octet-stream;charset=utf-8'
        response.headers['content-disposition'] = 'attachment;filename=systemLog.csv'
        return response


class DownloadBackLog(Resource):
    """ Download outside and wcs log """
    def __init__(self):
        self.schema = BackLogSchema()

    @staticmethod
    def getZipLogFile(date, check):
        log_file_name = '~{}.log'.format("".join(date.split("_")))
        outside_log_path = os.path.join(CMCT_LOG_PATH,"outside", log_file_name)
        wcs_log_path = os.path.join(CMCT_LOG_PATH,"wcs", log_file_name)

        if not os.path.exists(outside_log_path):
            msg = "outside log file [{}] not exists".format(outside_log_path)
            res = make_json_response(code=-1, msg=msg)
            return res
        
        if check:
            msg = "log file exists"
            res = make_json_response(code=0, msg=msg)
            return res
        
        zip_file_path =  os.path.join(CMCT_LOG_PATH, log_file_name.replace("log","zip"))

        # create zip file
        zip = zipfile.ZipFile(zip_file_path, "w", zipfile.ZIP_DEFLATED)
        zip.write(outside_log_path, 'outside'+log_file_name)
        if os.path.exists(wcs_log_path):
            zip.write(wcs_log_path, 'wcs'+log_file_name)
        zip.close()
        zip_data = None
        with open(zip_file_path, 'rb') as f:
            zip_data = f.read()


        response = make_response(zip_data)
        response.headers['content-type'] = 'application/zip'
        response.headers['content-disposition'] = 'attachment;filename=BackLog_{}.zip'.format(date)

        # remove zip file
        os.remove(zip_file_path)
        return response

    @req_log(ignoreReq=True, ignoreRes=True)
    @openapi.api_doc(tags=["XLHB(标准库接口汇总)", "日志"], summary="下载后端日志", request_body=BackLogSchema)
    def get(self):
        data, error = self.schema.loads(json.dumps(request.args.to_dict()))
        if error:
            msg = 'data type error {}'.format(error)
            res = make_json_response(code=-1, msg=msg)
            return res
        
        check = data.get("check", False)

        response = self.getZipLogFile(data['date'], check)
        return response

    @req_log(ignoreReq=True, ignoreRes=True)
    @openapi.api_doc(tags=["XLHB(标准库接口汇总)", "日志"], summary="下载后端日志", request_body=BackLogSchema)
    def post(self):
        data, error = self.schema.loads(request.data)

        if error:
            msg = 'data type error {}'.format(error)
            res = make_json_response(code=-1, msg=msg)
            return res

        check = data.get("check", False)

        response = self.getZipLogFile(data['date'], check)
        return response

class QueryLogMenu(Resource):
    def __init__(self):
        self.schema = QueryLogMenuSchema()

    @req_log(ignoreRes=True)
    @openapi.api_doc(tags=["XLHB(标准库接口汇总)", "日志"], summary="节点日志菜单", request_body=QueryLogMenuSchema)
    def post(self):
        """
        Returns:
            ---
            {"class": [], "tag": []}
            ---
        """
        error = None
        data, error = self.schema.loads(request.data)

        if error:
            msg = 'data type error'
            res = make_json_response(code=-1, msg=msg)
            return res

        star_timestamp = trans_timestamp(data['date'])
        star_date = datetime.datetime.strptime(data['date'], "%Y_%m_%d")
        end_date = (star_date + datetime.timedelta(days=1)).strftime('%Y_%m_%d')
        end_timestamp = trans_timestamp(str(end_date))

        res_data = {"class": [], "tag": []}
        for log in Log.query.filter(Log.timestamp.between(star_timestamp, end_timestamp)).with_entities(Log.class_).distinct(Log.class_):
            res_data["class"].extend(log)

        for log in Log.query.filter(Log.timestamp.between(star_timestamp, end_timestamp)).with_entities(Log.tag).distinct(Log.tag):
            res_data["tag"].extend(log)

        return make_json_response(data=res_data)


class QueryRobotStatus(Resource):
    """查询当前机器人的状态，如果状态查询失败，则机器人状态返回0
    """

    @req_log(ignoreReq=True)
    @openapi.api_doc(tags=["XLHB(标准库接口汇总)"], summary="查询机器人状态")
    def get(self):
        """
        Returns:
            ---
            {"code": 0, "message": "", "data": {"status": "ready"}}
            ---
        """
        status = get_robot_status()
        res_data = {
            "status": status.value 
        }
        return make_json_response(data=res_data)
