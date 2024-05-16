# -*- coding: utf-8 -*-
"""
Copyright (c) XYZ Robotics Inc. - All Rights Reserved
Unauthorized copying of this file, via any medium is strictly prohibited
Proprietary and confidential
Author: Yuhang Wu <yuhang.wu@xyzrobotics.ai>, 2022-07-05

用于测试历史任务相关的测试用例
"""
import copy
import unittest
from datetime import datetime, timedelta

import jsonschema
from flask import url_for

from apps import create_app
from wcs_adaptor.api.task_history.crud import crud_task_history
from wcs_adaptor.api.task_history.schema import Response


class TestTaskHistoryAPI(unittest.TestCase):

    def __init__(self, *args, **kwargs):
        super(TestTaskHistoryAPI, self).__init__(*args, **kwargs)
        self.app = create_app(testing=True)
        self.client = self.app.test_client()
        import wcs_adaptor
        wcs_adaptor.init_app(self.app)

    def remove_task(self):
        """清空任务."""
        resp = self.client.post("/api/cmd/remove_task")
        assert resp.json == {
            'code': 0,
            'data': {},
            'error': 0,
            'error_message': '',
            'msg': '',
        }, resp.json

    def create_task(self, task_type, task_id):
        """Create a task.

        Args:
            task_type(str): 任务类型
            task_id(str): 任务ID

        Returns:
            a response.

        """
        body = {
            "task_id": task_id,
            "sku_info": {
                "sku_id": "",
                "length": 0.21,
                "width": 0.14,
                "height": 0.105,
                "weight": 1.0,
                "sku_num": 1
            },
            "from": "0",
            "to": "1"
        }
        with self.app.test_client(), self.app.test_request_context():
            api = url_for(f"wcs.{task_type}")
            resp = self.client.post(api, json=body)
            return resp.json

    def report_task(self, task_id, num):
        with self.app.test_client(), self.app.test_request_context():
            api = url_for("rafcon.report_task_status")
            body = {
                "task_id": task_id,
                "pick_num": num,
                "is_depal_pallet_empty": False,
                "is_pal_pallet_full": False,
                "error": 0,
                "error_message": "success"
            }
            self.client.post(api, json=body)
            resp = self.client.post(url_for("rafcon.report_task_ending"), json={"task_id": task_id})
            assert resp.json["code"] == 0, resp.json

    def test_search_task(self):
        """测试根据搜索条件获取记录."""
        with self.app.app_context(), self.app.test_request_context():
            crud_task_history.delete_all()
            search_api = url_for("task_history.list_tasks")
            json = {
                "page": 1,
                "page_size": 20,
                "single": False
            }
            # 搜索空数据
            resp = self.client.post(search_api, json=json)
            assert resp.json["code"] == 0 and resp.json["count"] == 0 and \
                   resp.json["data"] == [], resp.json

            # 创建任务
            self.create_task("single_class_depal_task", "999")

            # 完成任务
            self.report_task(task_id="999", num=1)

            # 搜索历史任务
            resp = self.client.post(search_api, json=json)
            assert resp.json["code"] == 0 and resp.json[
                "total_page"] == 1 and len(resp.json["data"]) == 1 and \
                   resp.json["data"][0]["task_id"] == "999"

            # 搜索创建时间在1分钟前的历史任务.
            json2 = {
                "page": 1,
                "page_size": 20,
                "single": False,
                "filters": [
                    {
                        "op": "gt",
                        "field": "create_time",
                        "value": (
                            datetime.now() - timedelta(minutes=1)).strftime(
                            "%Y-%m-%d %HH:%MM:%SS"
                        )
                    }
                ]
            }
            resp = self.client.post(search_api, json=json2)
            print(resp.json)
            assert resp.json["code"] == 0 and resp.json[
                "total_page"] == 1 and len(resp.json["data"]) == 1 and \
                   resp.json["data"][0]["task_id"] == "999", resp.json

            # 搜索不满足搜索条件的历史任务.
            json3 = {
                "page": 1,
                "page_size": 20,
                "single": False,
                "filters": [
                    {
                        "op": "lt",
                        "field": "create_time",
                        "value": (
                            datetime.now() - timedelta(minutes=1)).strftime(
                            "%Y-%m-%d %HH:%MM:%SS"
                        )
                    }
                ]
            }
            resp = self.client.post(search_api, json=json3)
            assert len(resp.json["data"]) == 0 and resp.json[
                "count"] == 0, resp.json

            # 测试single参数
            json4 = copy.deepcopy(json2)
            json4["single"] = True
            resp = self.client.post(search_api, json=json4)
            assert isinstance(resp.json["data"], dict) and resp.json["data"][
                "task_id"] == "999", resp.json
            crud_task_history.delete_all()

    def test_get_task(self):
        """测试根据主键ID获取记录."""
        with self.app.test_client(), self.app.test_request_context():
            self.remove_task()
            crud_task_history.delete_all()
            api = url_for("task_history.get_task", id=777)
            # 获取不存在的任务.
            resp = self.client.get(api)
            assert resp.json["code"] == -1 and resp.json[
                "msg"] == "Record Not Found"

            # 创建任务
            self.create_task("single_class_depal_task", "777")

            # 完成任务
            self.report_task(task_id="777", num=1)

            # 获取已存在的任务
            task = crud_task_history.last()
            api = url_for("task_history.get_task", id=task.id)
            resp = self.client.get(api)
            print(resp.json)
            assert resp.json["code"] == 0, jsonschema.validate(
                resp.json,
                Response.schema()
            )

            self.remove_task()
            crud_task_history.delete_all()

    def test_download_file(self):
        """测试文件下载."""
        with self.app.test_client(), self.app.test_request_context():
            self.remove_task()
            crud_task_history.delete_all()

            # 创建任务
            self.create_task("single_class_depal_task", "777")

            # 完成任务
            self.report_task(task_id="777", num=1)

            # 下载历史任务
            resp = self.client.get(url_for("task_history.download_task"))

            # filename = resp.headers[0][1].rsplit("=", 1)[-1]
            # with open(f"./{filename}", "wb") as f:
            #     f.write(resp.data)
            self.remove_task()
            crud_task_history.delete_all()
