# -*- coding: utf-8 -*-
"""
Copyright (c) XYZ Robotics Inc. - All Rights Reserved
Unauthorized copying of this file, via any medium is strictly prohibited
Proprietary and confidential
Author: Yuhang Wu <yuhang.wu@xyzrobotics.ai>, 2022-07-27

用于垛型规划中的纸箱接口.
"""
import json
import unittest
import uuid
from typing import List

from flask import url_for

from apps import create_app
from apps.models import start_transaction
from apps.programs.dpt.views.design_v2.crud import crud_box, crud_plan
from apps.programs.dpt.views.design_v2.schemas import PlanCreateSchema

url_name = "dpt.design.v2.box"


class TestBoxAPI(unittest.TestCase):
    def setUp(self) -> None:
        self.app = create_app(testing=True)
        self.client = self.app.test_client()
        with self.app.app_context(), self.app.test_request_context():
            self.clear_boxes()

    def create_box(self, data: dict):
        """创建纸箱."""
        return self.client.post(
            url_for(f"{url_name}.create_box"),
            # mimetype="application/form-data",
            data=data,
        )

    def create_plan(self, data: dict):
        """创建垛型规划."""
        resp = self.client.post(
            url_for(f"dpt.design.v2.plan.add_plan"),
            data=data,
        )
        assert resp.json["code"] == 0, resp.json
        return resp.json["data"]

    def delete_box(self, _id: int):
        """删除纸箱."""
        return self.client.delete(url_for(f"{url_name}.delete_box", id=_id))

    def delete_boxes(self, ids: List[int]):
        """删除多个纸箱记录."""
        return self.client.delete(
            url_for(f"{url_name}.delete_boxes"), json={"ids": ids}
        )

    def clear_boxes(self):
        """清空纸箱."""
        return self.client.delete(
            url_for(f"{url_name}.clear_all")
        )

    def test_create_box(self):
        """测试创建纸箱."""
        name = uuid.uuid4().__str__()
        # 正常创建.
        data = {
            "name": name,
            "length": 1000.0,
            "width": 1000.0,
            "height": 1000.0,
            "weight": 30,
            "scan_code": name.split("-", 1)[0],
        }
        with self.app.app_context(), self.app.test_request_context():
            resp = self.create_box(data)
            success_data = resp.json["data"]
            assert resp.json["code"] == 0, resp.json
            assert resp.json["data"]["name"] == name, resp.json

            # 重复入参
            resp = self.create_box(data)
            assert resp.json["code"] == -1, resp.json
            assert resp.json["msg"] == "Integrity Error", resp.json

            # 错误的入参
            data = {"name": uuid.uuid4().__str__()}
            resp = self.create_box(data)
            assert resp.json["code"] != 0, resp.json

            # 清空数据.
            self.delete_box(success_data["id"])

    def test_delete_box(self):
        """测试删除纸箱."""
        with self.app.app_context(), self.app.test_request_context():
            name = uuid.uuid4().__str__()
            # 正常创建.
            data = {
                "name": name,
                "length": 1000.0,
                "width": 1000.0,
                "height": 1000.0,
                "weight": 30,
                "scan_code": name.split("-", 1)[0],
            }
            resp = self.create_box(data)
            success_data = resp.json["data"]

            # 删除数据.
            self.delete_box(success_data["id"])

            # 查询数据.
            pk = success_data["id"]
            result = crud_box.get(pk=pk)
            assert result.is_del is None, f"纸箱数据未删除, id: {pk}"

    def test_delete_box_with_design(self):
        with self.app.app_context(), self.app.test_request_context():
            with start_transaction() as session:
                # 创建纸箱
                data = {
                    "name": uuid.uuid4().__str__(),
                    "length": 1000.0,
                    "width": 1000.0,
                    "height": 1000.0,
                    "weight": 30,
                    "scan_code": uuid.uuid4().__str__(),
                }
                resp = self.create_box(data)
                assert resp.json["code"] == 0, resp.json
                box_id = resp.json["data"]["id"]

                # 创建深框.
                data = {
                    "name": uuid.uuid4().__str__(),
                    "length": 2000.0,
                    "width": 1500.0,
                    "height": 2000.0,
                    "max_height": 20000,
                    "max_weight": 300,
                    "pallet_type": True,
                    "thickness": 100,
                }
                resp = self.client.post(
                    url_for("dpt.design.v2.pallet.create_pallet"), json=data
                )
                assert resp.json["code"] == 0, resp.json
                pallet_id = resp.json["data"]["id"]

                plan = self.create_plan(
                    {
                        "pallet_id": pallet_id,
                        "box_id": box_id,
                        "guillotine_packing": 0,
                        "barcode_direction": 0,
                        "mirror": 0,
                        "flip": "x",
                        "layers": 0,
                        # "layout": [[], []],
                        "objects": json.dumps({
                            "odd_layer": [
                                {
                                    "label": 1,
                                    "x": 499.99997999999994,
                                    "y": 0.000015000000075815478,
                                    "rotation": 1,
                                    "place_drop_buffer": 0,
                                },
                                {
                                    "label": 2,
                                    "x": -500.00002000000006,
                                    "y": 0.000015000000075815478,
                                    "rotation": 1,
                                    "place_drop_buffer": 0,
                                },
                            ],
                            "even_layer": [
                                {
                                    "label": 1,
                                    "x": 499.99997999999994,
                                    "y": 0.000015000000075815478,
                                    "rotation": 1,
                                    "place_drop_buffer": 0,
                                },
                                {
                                    "label": 2,
                                    "x": -500.00002000000006,
                                    "y": 0.000015000000075815478,
                                    "rotation": 1,
                                    "place_drop_buffer": 0,
                                },
                            ],
                        })
                    }
                )

                # 删除纸箱, 删除失败
                resp = self.delete_box(box_id)
                assert (
                    resp.json["code"] == -1 and resp.json["data"]["error_code"] == 10500
                ), resp.json

                # 删除规划记录
                crud_plan.delete(session=session, pk=plan["id"])

                # 再次删除纸箱，删除成功
                resp = self.delete_box(box_id)
                assert resp.json["code"] == 0, resp.json

    def test_delete_boxes_with_design(self):
        """删除多个纸箱记录."""
        with self.app.app_context(), self.app.test_request_context():
            with start_transaction() as session:
                # 创建纸箱
                data = {
                    "name": uuid.uuid4().__str__(),
                    "length": 1000.0,
                    "width": 1000.0,
                    "height": 1000.0,
                    "weight": 30,
                    "scan_code": uuid.uuid4().__str__(),
                }
                resp = self.create_box(data)
                assert resp.json["code"] == 0, resp.json
                box_id = resp.json["data"]["id"]

                # 创建深框.
                data = {
                    "name": uuid.uuid4().__str__(),
                    "max_height": 20000,
                    "length": 2000.0,
                    "width": 1500.0,
                    "height": 2000.0,
                    "max_weight": 300,
                    "pallet_type": True,
                    "thickness": 100,
                }
                resp = self.client.post(
                    url_for("dpt.design.v2.pallet.create_pallet"), json=data
                )
                assert resp.json["code"] == 0, resp.json
                pallet_id = resp.json["data"]["id"]

                plan = self.create_plan(
                    {
                        "pallet_id": pallet_id,
                        "box_id": box_id,
                        "guillotine_packing": 0,
                        "barcode_direction": 0,
                        "mirror": 0,
                        "flip": "x",
                        "layers": 0,
                        # "layout": [[], []],
                        "objects": json.dumps({
                            "odd_layer": [
                                {
                                    "label": 1,
                                    "x": 499.99997999999994,
                                    "y": 0.000015000000075815478,
                                    "rotation": 1,
                                    "place_drop_buffer": 0,
                                },
                                {
                                    "label": 2,
                                    "x": -500.00002000000006,
                                    "y": 0.000015000000075815478,
                                    "rotation": 1,
                                    "place_drop_buffer": 0,
                                },
                            ],
                            "even_layer": [
                                {
                                    "label": 1,
                                    "x": 499.99997999999994,
                                    "y": 0.000015000000075815478,
                                    "rotation": 1,
                                    "place_drop_buffer": 0,
                                },
                                {
                                    "label": 2,
                                    "x": -500.00002000000006,
                                    "y": 0.000015000000075815478,
                                    "rotation": 1,
                                    "place_drop_buffer": 0,
                                },
                            ],
                        })
                    }
                )

                # 删除纸箱, 删除失败, data 是空的
                resp = self.delete_boxes([box_id])
                assert resp.json["code"] == 0, resp.json
                assert resp.json["data"] == [], resp.json

                # 删除规划记录
                # 逻辑删除后，规划记录仍然存在表里，此时删除纸箱记录，会将规划记录所绑定的box_id置为None
                # 但是 box_id 是不能为 None 的，所以删除失败
                crud_plan.delete(session=session, pk=plan["id"])

                # 再次删除纸箱，删除成功
                resp = self.delete_boxes([box_id])
                assert resp.json["code"] == 0, resp.json
                assert len(resp.json["data"]) == 1, resp.json

    def test_clear_all_with_design(self):
        """测试清空正在被使用的纸箱."""
        with self.app.app_context(), self.app.test_request_context():
            with start_transaction() as session:
                # 创建纸箱
                data = {
                    "name": uuid.uuid4().__str__(),
                    "length": 1000.0,
                    "width": 1000.0,
                    "height": 1000.0,
                    "weight": 30,
                    "scan_code": uuid.uuid4().__str__(),
                }
                resp = self.create_box(data)
                assert resp.json["code"] == 0, resp.json
                box_id = resp.json["data"]["id"]

                # 创建深框.
                data = {
                    "name": uuid.uuid4().__str__(),
                    "max_height": 20000,
                    "length": 2000.0,
                    "width": 1500.0,
                    "height": 2000.0,
                    "max_weight": 300,
                    "pallet_type": True,
                    "thickness": 100,
                }
                resp = self.client.post(
                    url_for("dpt.design.v2.pallet.create_pallet"), json=data
                )
                assert resp.json["code"] == 0, resp.json
                pallet_id = resp.json["data"]["id"]

                plan = self.create_plan(
                    {
                        "pallet_id": pallet_id,
                        "box_id": box_id,
                        "guillotine_packing": 0,
                        "barcode_direction": 0,
                        "mirror": 0,
                        "flip": "x",
                        "layers": 0,
                        # "layout": [[], []],
                        "objects": json.dumps({
                            "odd_layer": [
                                {
                                    "label": 1,
                                    "x": 499.99997999999994,
                                    "y": 0.000015000000075815478,
                                    "rotation": 1,
                                    "place_drop_buffer": 0,
                                },
                                {
                                    "label": 2,
                                    "x": -500.00002000000006,
                                    "y": 0.000015000000075815478,
                                    "rotation": 1,
                                    "place_drop_buffer": 0,
                                },
                            ],
                            "even_layer": [
                                {
                                    "label": 1,
                                    "x": 499.99997999999994,
                                    "y": 0.000015000000075815478,
                                    "rotation": 1,
                                    "place_drop_buffer": 0,
                                },
                                {
                                    "label": 2,
                                    "x": -500.00002000000006,
                                    "y": 0.000015000000075815478,
                                    "rotation": 1,
                                    "place_drop_buffer": 0,
                                },
                            ],
                        })
                    }
                )

                # 清空纸箱, 全部处于占用状态, 预期成功删除 0 条
                resp = self.clear_boxes()
                assert resp.json["code"] == 0 and resp.json["data"]["deleted_count"] == 0, resp.json

                # 删除规划记录
                # 逻辑删除后，规划记录仍然存在表里，此时删除纸箱记录，会将规划记录所绑定的box_id置为None
                # 但是 box_id 是不能为 None 的，所以删除失败
                crud_plan.delete(session=session, pk=plan["id"])

                # 清空纸箱, 未处于占用状态, 预期成功删除 1 条
                resp = self.clear_boxes()
                assert resp.json["code"] == 0 and resp.json["data"]["deleted_count"] == 1, resp.json

    def test_parse_box(self):
        """测试解析表格文件."""

        # 上传标准表格文件.

        # 上传长宽高重复表格文件.

        # 上传字段缺失的表格文件.

        # TODO(YuhangWu): 在此issue中补充， https://xyz-robotics.atlassian.net/browse/MQWR-1148
        pass

    def test_download_file(self):
        """测试下载纸箱模板文件."""
        # TODO(YuhangWu): 在此issue中补充， https://xyz-robotics.atlassian.net/browse/MQWR-1148
        pass
