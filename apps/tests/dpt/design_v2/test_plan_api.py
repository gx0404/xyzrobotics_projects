# -*- coding: utf-8 -*-
"""
Copyright (c) XYZ Robotics Inc. - All Rights Reserved
Unauthorized copying of this file, via any medium is strictly prohibited
Proprietary and confidential
Author: Yuhang Wu <yuhang.wu@xyzrobotics.ai>, 2022-07-28

用于垛型规划的测试用例.
"""
import json
import os
import unittest
import uuid
from pathlib import Path
from typing import Optional

from flask import url_for

from apps import create_app
from apps.models import start_transaction
from apps.programs.dpt.views.design.models import Design as Plan
from apps.programs.dpt.views.design_v2.crud import crud_box, crud_pallet
from apps.programs.dpt.views.design_v2.schemas import (
    BoxCreateSchema,
    PalletCreateSchema,
)

url_name = "dpt.design.v2.plan"


class TestPlanAPI(unittest.TestCase):
    def setUp(self) -> None:
        self.app = create_app(testing=True)
        self.client = self.app.test_client()

    def tearDown(self) -> None:
        # 清空数据表
        with self.app.app_context(), self.app.test_request_context():
            with start_transaction() as session:
                session.query(Plan).filter().delete()

    @staticmethod
    def create_box(
        name: Optional[str] = None,
        length: int = 1000,
        width: int = 1000,
        height: int = 1000,
        weight: int = 20,
        scan_code: str = None,
        img_file: str = None,
    ):
        if name is None:
            name = str(uuid.uuid4())
        with start_transaction() as session:
            box = crud_box.create(
                session=session,
                create=BoxCreateSchema(
                    name=name,
                    length=length,
                    width=width,
                    height=height,
                    weight=weight,
                    scan_code=scan_code,
                    img_file=img_file,
                ),
            )
        return box

    @staticmethod
    def create_pallet(
        name: Optional[str] = None,
        length: int = 2000,
        width: int = 1500,
        height: int = 2000,
        max_height: int = 3000,
        max_weight: int = 10000,
        pallet_type: bool = False,
    ):
        if name is None:
            name = str(uuid.uuid4())
        with start_transaction() as session:
            pallet = crud_pallet.create(
                session=session,
                create=PalletCreateSchema(
                    name=name,
                    length=length,
                    width=width,
                    height=height,
                    max_height=max_height,
                    max_weight=max_weight,
                    pallet_type=pallet_type,
                ),
            )
        return pallet

    @staticmethod
    def create_box_and_pallet():
        with start_transaction() as session:
            box = crud_box.create(
                session=session,
                create=BoxCreateSchema(
                    name=str(uuid.uuid4()),
                    length=1000,
                    width=1000,
                    height=1000,
                    weight=20,
                    scan_code=None,
                    img_file=None,
                ),
            )
            pallet = crud_pallet.create(
                session=session,
                create=PalletCreateSchema(
                    name=str(uuid.uuid4()),
                    length=2000,
                    width=1500,
                    height=2000,
                    max_height=3000,
                    max_weight=10000,
                    pallet_type=False,
                ),
            )
            return box, pallet

    def generate_plan(self,
        box,
        pallet,
        guillotine_packing: int = 0,
        barcode_direction: int = 0,
        mirror: int = 0,
        flip: str = "x",
        layers: int = 0,
    ):
        data = {
            "pallet_id": pallet.id,
            "box_id": box.id,
            "guillotine_packing": guillotine_packing,
            "barcode_direction": barcode_direction,
            "mirror": mirror,
            "flip": flip,
            "layers": layers,
        }
        resp = self.client.post(
            url_for(f"{url_name}.generate_plan"),
            json=data,
        )
        assert resp.json["code"] == 0, resp.json
        return resp.json["data"]


    def test_generate_plan(self):
        """测试生成一个规划记录."""
        with self.app.app_context(), self.app.test_request_context():
            box, pallet = self.create_box_and_pallet()
            data = {
                "pallet_id": pallet.id,
                "box_id": box.id,
                "guillotine_packing": 0,
                "barcode_direction": 0,
                "mirror": 0,
                "flip": "x",
                "layers": 0,
            }
            resp = self.client.post(
                url_for(f"{url_name}.generate_plan"),
                json=data,
            )
            assert resp.json["code"] == 0, resp.json
            return resp.json["data"]

    def test_generate_plans(self):
        """测试批量生成规划记录."""
        with self.app.app_context(), self.app.test_request_context():
            box, pallet = self.create_box_and_pallet()
            resp = self.client.post(
                url_for(f"{url_name}.generate_plans"),
                data={
                    "pallet_id": pallet.id,
                    "box_id": box.id,
                    "guillotine_packing": 0,
                    "barcode_direction": 0,
                    "mirror": 0,
                    "flip": "x",
                    "layers": 0,
                    "box_file": open(Path(Path(__file__).parent, "box.xlsx"), "rb"),
                },
            )
            assert resp.json["code"] == 0, resp.json

    def test_get_one_plan(self):
        """测试获取一个规划记录."""
        # 生成一个规划记录.
        data = self.test_add_plan()

        with self.app.app_context(), self.app.test_request_context():
            resp = self.client.get(url_for(f"{url_name}.get_plan", id=data["id"]))
            assert resp.json["code"] == 0, resp.json

    def test_add_plan(self):
        """测试添加一个规划记录."""
        # 生成一个规划记录.
        data = self.test_generate_plan()
        filename = Path(os.path.abspath(__file__)).parent / "1.png"
        # 基于这个data修改.
        new_data = {
            "pallet_id": data["pallet"]["id"],
            "box_id": data["box"]["id"],
            "guillotine_packing": data["guillotine_packing"],
            "barcode_direction": data["barcode_direction"],
            "mirror": data["mirror"],
            "flip": data["flip"],

            "layers": data["layers"],
            # "layout": data["layout"].__str__(),
            "objects": json.dumps(data["objects"]),
            "image": filename.open("rb"),
        }

        with self.app.app_context(), self.app.test_request_context():
            # 正常创建.
            resp = self.client.post(url_for(f"{url_name}.add_plan"), data=new_data)
            assert resp.json["code"] == 0, resp.json
            return resp.json["data"]

    def test_add_plan_with_errors(self):
        # 生成一个规划记录.
        data = self.test_generate_plan()
        # 构造一个错误的数据.
        new_data = {
            "pallet_id": data["pallet"]["id"],
            "box_id": data["box"]["id"],
            "guillotine_packing": data["guillotine_packing"],
            "barcode_direction": data["barcode_direction"],
            "mirror": data["mirror"],
            "flip": data["flip"],
            "layers": data["layers"],
            "layout": "[]",
        }

        with self.app.app_context(), self.app.test_request_context():
            resp = self.client.post(url_for(f"{url_name}.add_plan"), data=new_data)
            assert resp.json["code"] != 0, resp.json
            # assert (
            #     resp.json["data"]["error_message"] == "layout format error"
            # ), resp.json

    def test_add_plan_no_image(self):
        # 生成一个规划记录.
        data = self.test_generate_plan()
        # 基于这个data修改.
        new_data = {
            "pallet_id": data["pallet"]["id"],
            "box_id": data["box"]["id"],
            "guillotine_packing": data["guillotine_packing"],
            "barcode_direction": data["barcode_direction"],
            "mirror": data["mirror"],
            "flip": data["flip"],
            "layers": data["layers"],
            # "layout": data["layout"].__str__(),
            "objects": json.dumps(data["objects"])
        }

        with self.app.app_context(), self.app.test_request_context():
            # 正常创建.
            resp = self.client.post(url_for(f"{url_name}.add_plan"), data=new_data)
            assert resp.json["code"] == 0, resp.json
            return resp.json["data"]

    def test_add_plan_incorrect_layers_and_height(self):
        """测试错误的自定义层数与纸箱高度."""
        with self.app.app_context(), self.app.test_request_context():
            box = self.create_box(height=1000)
            pallet = self.create_pallet(max_height=1000)
            data = self.generate_plan(box=box, pallet=pallet, layers=2)
            new_data = {
                "pallet_id": data["pallet"]["id"],
                "box_id": data["box"]["id"],
                "guillotine_packing": data["guillotine_packing"],
                "barcode_direction": data["barcode_direction"],
                "mirror": data["mirror"],
                "flip": data["flip"],
                "layers": data["layers"],
                # "layout": data["layout"].__str__(),
                "objects": json.dumps(data["objects"]),
            }
            # 正常创建.
            resp = self.client.post(url_for(f"{url_name}.add_plan"), data=new_data)
            assert resp.json["code"] != 0, resp.json

    def test_list_no_image_plans(self):
        """测出获取无图片记录."""
        data = self.test_add_plan_no_image()
        with self.app.app_context(), self.app.test_request_context():
            resp = self.client.get(url_for(f"{url_name}.list_no_image_plans"))
            assert resp.json["code"] == 0, resp.json
            assert resp.json["data"][0]["id"] == data["id"], resp.json

    def test_edit_plan(self):
        """测试更新一个规划记录."""
        # 生成一个规划记录.
        data = self.test_add_plan()

        # 基于这个data修改.
        new_data = {
            "pallet_id": data["pallet"]["id"],
            "box_id": data["box"]["id"],
            "guillotine_packing": data["guillotine_packing"],
            "barcode_direction": data["barcode_direction"],
            "mirror": data["mirror"],
            "flip": data["flip"],
            "layers": 1,
            # "layout": json.dumps(data["layout"]),
            "objects": json.dumps(data["objects"])
        }
        with self.app.app_context(), self.app.test_request_context():
            resp = self.client.put(
                url_for(f"{url_name}.edit_plan", id=data["id"]), data=new_data
            )
            assert resp.json["code"] == 0, resp.json
            # 不允许修改 layers, 其值应该与原来的一致
            assert resp.json["data"]["layers"] != new_data["layers"]

    def test_delete_one_plan(self):
        """测试删除一个规划记录."""
        data = self.test_add_plan()
        with self.app.app_context(), self.app.test_request_context():
            resp = self.client.delete(url_for(f"{url_name}.delete_plan", id=data["id"]))
            assert resp.json["code"] == 0, resp.json
            assert resp.json["data"]["id"] == data["id"], resp.json

    def test_delete_multiple_plan(self):
        """删除多个规划记录."""
        ids = [self.test_add_plan()["id"] for _ in range(3)]
        with self.app.app_context(), self.app.test_request_context():
            resp = self.client.delete(
                url_for(f"{url_name}.delete_plans"), json={"ids": ids}
            )
            assert resp.json["code"] == 0, resp.json
            deleted_ids = [data["id"] for data in resp.json["data"]]
        assert ids == deleted_ids

    def test_clear_all(self):
        """清空垛型."""
        _ = [self.test_add_plan()["id"] for _ in range(3)]
        with self.app.app_context(), self.app.test_request_context():
            resp = self.client.delete(
                url_for(f"{url_name}.clear_all")
            )
            assert resp.json["code"] == 0 and resp.json["data"]["deleted_count"] == 3, resp.json

    def test_search_plan(self):
        """测试规划记录搜索接口."""
        data = self.test_add_plan()
        with self.app.app_context(), self.app.test_request_context():
            # 测试模糊查询纸箱名
            js = {
                "filters": [
                    {
                        "op": "contains",
                        "field": "box_name",
                        "value": data["box"]["name"][1:6],
                    }
                ]
            }
            resp = self.client.post(url_for(f"{url_name}.list_plans"), json=js)
            assert resp.json["code"] == 0, resp.json
            assert resp.json["data"][0]["id"] == data["id"], resp.json

            js["filters"][0]["value"] = "xxxxxxxxxxxxxxxxxxx"
            resp = self.client.post(url_for(f"{url_name}.list_plans"), json=js)

            # 预期无数据
            assert resp.json["code"] == 0, resp.json
            assert resp.json["count"] == 0, resp.json

            # 根据箱子的长宽高匹配规划记录.
            js = {
                "filters": [
                    {
                        "op": "gt",
                        "field": "length",
                        "value": data["box"]["length"] - 100,
                    }
                ]
            }
            resp = self.client.post(url_for(f"{url_name}.list_plans"), json=js)
            assert resp.json["code"] == 0, resp.json
            assert resp.json["data"][0]["id"] == data["id"], resp.json
