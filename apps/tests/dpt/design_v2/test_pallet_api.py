# -*- coding: utf-8 -*-
"""
Copyright (c) XYZ Robotics Inc. - All Rights Reserved
Unauthorized copying of this file, via any medium is strictly prohibited
Proprietary and confidential
Author: Yuhang Wu <yuhang.wu@xyzrobotics.ai>, 2022-07-27

用于垛型规划中的托盘接口.
"""
import unittest
import uuid

from flask import url_for

from apps import create_app
from apps.models import start_transaction
from apps.programs.dpt.views.design_v2.crud import crud_pallet, crud_plan
from apps.programs.dpt.views.design_v2.schemas import PlanCreateSchema

url_name = "dpt.design.v2.pallet"


class TestPalletAPI(unittest.TestCase):

    def setUp(self) -> None:
        self.app = create_app(testing=True)
        self.client = self.app.test_client()

    def create_pallet(self, data: dict):
        """创建托盘."""
        return self.client.post(
            url_for(f"{url_name}.create_pallet"),
            json=data
        )

    def delete_pallet(self, _id: int):
        """删除托盘."""
        return self.client.delete(url_for(f"{url_name}.delete_pallet", id=_id))

    def test_create_pallet(self):
        """测试创建托盘."""
        name = uuid.uuid4().__str__()

        # 创建深框.
        data = {
            "name": name,
            "length": 1000.0,
            "width": 1000.0,
            "height": 1000.0,
            "max_height": 20000,
            "max_weight": 300,
            "pallet_type": True,
            "thickness": 100,
        }
        with self.app.app_context(), self.app.test_request_context():
            resp = self.create_pallet(data)
            success_data = resp.json["data"]
            assert resp.json["code"] == 0, resp.json
            assert resp.json["data"]["name"] == name, resp.json

            # 重复入参
            resp = self.create_pallet(data)
            assert resp.json["code"] == -1, resp.json
            assert resp.json["msg"] == "Integrity Error", resp.json

            # 错误的入参
            data = {"name": uuid.uuid4().__str__()}
            resp = self.create_pallet(data)
            assert resp.json["code"] != 0, resp.json

            # 清空数据.
            self.delete_pallet(success_data["id"])

            # 创建托盘
            data = {
                "name": uuid.uuid4().__str__(),
                "length": 1000.0,
                "width": 1000.0,
                "height": 1000.0,
                "max_height": 20000,
                "max_weight": 300,
                "pallet_type": False,
                # 托盘指定了thickness, 预期效果不生效，即该字段将返回None.
                "thickness": 100,
            }
            resp = self.create_pallet(data)
            assert resp.json["code"] == 0, resp.json
            assert resp.json["data"]["thickness"] is None, resp.json["data"]

    def test_delete_pallet(self):
        """测试删除托盘."""
        with self.app.app_context(), self.app.test_request_context():
            name = uuid.uuid4().__str__()
            # 正常创建.
            data = {
                "name": name,
                "length": 1000.0,
                "width": 1000.0,
                "height": 1000.0,
                "max_height": 20000,
                "max_weight": 300,
                "pallet_type": True,
                "thickness": 100,
            }
            resp = self.create_pallet(data)
            success_data = resp.json["data"]

            # 删除数据.
            resp = self.delete_pallet(success_data["id"])
            assert resp.json["code"] == 0, resp.json
            # 查询数据.
            pk = success_data["id"]
            result = crud_pallet.get(pk=pk)
            assert result.is_del is None, f"纸箱数据未删除, id: {pk}"

    def test_delete_pallet_with_design(self):
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
                resp = self.client.post(
                    url_for(f"dpt.design.v2.box.create_box"),
                    data=data,
                )
                assert resp.json["code"] == 0, resp.json
                box_id = resp.json["data"]["id"]

                # 创建深框.
                data = {
                    "name": uuid.uuid4().__str__(),
                    "length": 1000.0,
                    "width": 1000.0,
                    "height": 1000.0,
                    "max_height": 20000,
                    "max_weight": 300,
                    "pallet_type": True,
                    "thickness": 100,
                }
                resp = self.client.post(
                    url_for(f"dpt.design.v2.pallet.create_pallet"),
                    json=data
                )
                assert resp.json["code"] == 0, resp.json
                pallet_id = resp.json["data"]["id"]

                # 创建规划记录
                plan = crud_plan.create(
                    session=session,
                    create=PlanCreateSchema(
                        **{
                            "pallet_id": pallet_id,
                            "box_id": box_id,
                            "guillotine_packing": 0,
                            "barcode_direction": 0,
                            "mirror": 0,
                            "flip": "x",
                            "layers": 0,
                            "layout": [[], []]
                        }
                    ),
                )

                # 删除失败
                resp = self.delete_pallet(pallet_id)
                assert resp.json["code"] == -1 and resp.json["data"]["error_code"] == 10500, resp.json

                # 删除规划记录
                crud_plan.delete(session=session, pk=plan.id)

                # 删除成功
                resp = self.delete_pallet(pallet_id)
                assert resp.json["code"] == 0, resp.json

    def test_edit_pallet(self):
        """测试编辑托盘."""
        with self.app.app_context(), self.app.test_request_context():
            name = uuid.uuid4().__str__()
            # 正常创建.
            data = {
                "name": name,
                "length": 1000.0,
                "width": 1000.0,
                "height": 1000.0,
                "max_height": 20000,
                "max_weight": 300,
                "pallet_type": True,
                "thickness": 100,
            }
            resp = self.create_pallet(data)
            assert resp.json["code"] == 0
            success_data = resp.json["data"]

            # 更新数据.
            data["length"] = 2000.0
            resp = self.client.put(
                url_for(f"{url_name}.edit_pallet", id=success_data["id"]),
                json=data
            )
            assert resp.json["code"] == 0, resp.json
            assert resp.json["data"]["length"] == 2000, resp.json
