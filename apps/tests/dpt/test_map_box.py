# -*- coding: utf-8 -*-
"""
Copyright (c) XYZ Robotics Inc. - All Rights Reserved
Unauthorized copying of this file, via any medium is strictly prohibited
Proprietary and confidential
Author: Yuhang Wu <yuhang.wu@xyzrobotics.ai>, 2022-04-15
"""
import copy
import random
import time
import unittest
import uuid

import jsonschema
import requests
from jsonschema.exceptions import ValidationError

from apps.programs.dpt.views.map_box.api import bp
from apps.programs.dpt.views.map_box.schemas import (
    SingleResponse,
    MapBoxDeleteSchema,
    MapBoxOuterSchema as MapBox,
    MapBoxQuerySchema
)
from apps.search_filter import QueryFilter

client = requests.session()
host = "http://localhost:7002"
# "http://localhost:7002" + "/api/apt/map_box"
url_prefix = host + bp.url_prefix

# 生成唯一的纸箱编号
serial_number = uuid.uuid4().__str__()


# serial_number = '15c2003b-bd40-46fe-96dc-1e8798edac98'


def get_random():
    return random.randint(1, 100)


class TestMapBox(unittest.TestCase):
    """
    Test map_box api.
    """

    created_data = {
        "serial_number": serial_number,
        "normal_length": get_random(),
        "normal_width": get_random(),
        "normal_height": get_random(),
        "real_length": get_random(),
        "real_width": get_random(),
        "real_height": get_random(),
        "normal_weight": get_random(),
        "real_weight": get_random()
    }
    # 存储已创建的数据集
    created_data_list = []

    def setUp(self) -> None:
        self.create()

    def test_create(self):
        """Test create a record.

        Returns:

        """
        api, resp = self.create()
        try:
            jsonschema.validate(resp, SingleResponse.schema())
        except ValidationError as err:
            print("验证失败, resp: ", resp)
            raise err

        # 再次添加同一条记录, 查看是否抛出重复异常
        resp_raw = client.post(api, json=self.created_data)
        resp = resp_raw.json()
        assert resp == {
            'code': -1,
            'msg': 'Integrity Error',
            'data': {
                'error_code': 10500,
                'error_message': '添加失败(纸箱已存在), 请检查是否存在重复的长宽高或纸箱编号'
            }
        }, resp

        # 再次添加同样的长宽高
        data = copy.deepcopy(self.created_data)
        data["serial_number"] = uuid.uuid4().__str__()
        resp_raw = client.post(api, json=data)
        resp = resp_raw.json()
        assert resp == {
            'code': -1,
            'msg': 'Integrity Error',
            'data': {
                'error_code': 10500,
                'error_message': '添加失败(纸箱已存在), 请检查是否存在重复的长宽高或纸箱编号'
            }
        }, resp

    def create(self, data=None):
        api = url_prefix + "/add"
        resp_raw = client.post(api, json=data or self.created_data)
        resp = resp_raw.json()
        return api, resp

    def test_get_one_record_by_serial_number(self):
        """Test get a record by serial_number.

        Returns:

        """
        self.create()
        api = url_prefix + "/"
        filters = [
            QueryFilter(op="eq", field="serial_number", value=serial_number)
        ]
        body = MapBoxQuerySchema(
            page=1,
            page_size=20,
            single=True,
            filters=filters
        )
        resp_raw = client.get(api, json=body.dict())
        resp = resp_raw.json()
        # 校验schema
        try:
            jsonschema.validate(resp, SingleResponse.schema())
        except ValidationError as err:
            print("响应的数据结构校验失败", resp)
            raise err

        # 校验当前纸箱数据与上一步创建的纸箱数据是否一致
        cur_data = resp["data"]
        assert self.created_data["serial_number"] == cur_data["serial_number"]

    def test_invalid_search_operator(self):
        """

        Returns:

        """
        op = "xx"
        filters = [QueryFilter(op=op, field="serial_number", value="9")]
        body = MapBoxQuerySchema(
            page=1,
            page_size=20,
            single=False,
            filters=filters,
            sort=[]
        )
        resp_raw = client.get(url_prefix + "/", json=body.dict())
        resp = resp_raw.json()
        assert resp == {
            'code': -1,
            'msg': 'Unknown Search Operator.',
            'data': {'error_code': 10401, 'error_message': 'xx, 未知操作符'}
        }, resp

    def test_get_non_existent(self, number: str = "00000"):
        """Test get a non-existent record.

        Args:
            number(str): 纸箱编号

        Returns:

        """
        filters = [QueryFilter(op="eq", field="serial_number", value=number)]
        body = MapBoxQuerySchema(
            page=1,
            page_size=20,
            single=True,
            filters=filters,
            sort=[]
        )
        resp_raw = client.get(url_prefix + "/", json=body.dict())
        resp = resp_raw.json()
        assert resp["code"] == 0, resp


    def test_search(self):
        """Test get the data through a range query.

        Returns:

        """
        from apps.programs.dpt.views.map_box.schemas import PaginationResponse
        from apps.programs.dpt.views.map_box.schemas import MapBoxQuerySchema

        api = url_prefix + "/"
        # 1. 不带任何参数
        resp_raw = client.get(api, json={})
        resp = resp_raw.json()
        jsonschema.validate(resp, PaginationResponse.schema())

        # 2. 范围匹配, x - 1 < x < x + 1
        filters = [
            QueryFilter(
                op="gt",
                field="normal_length",
                value=self.created_data["normal_length"] - 1,
            ),
            QueryFilter(
                op="lt",
                field="normal_length",
                value=self.created_data["normal_length"] + 1,
            ),
        ]
        body = MapBoxQuerySchema(
            page=1,
            page_size=20,
            single=False,
            filters=filters,
        )
        resp_raw = client.get(api, json=body.dict())
        resp = resp_raw.json()
        jsonschema.validate(resp, PaginationResponse.schema())
        # 判断data是否有值
        assert resp["data"] != []
        # 查看匹配的data集合中是否有self.created_data
        exits = False
        for d in resp["data"]:
            if d["serial_number"] == self.created_data["serial_number"]:
                exits = True
        assert exits is True, "范围匹配测试未通过"

        # 3. 模糊匹配纸箱编号, where field like "123%"
        number = self.created_data.get("serial_number")[:4]
        # 截取编号的前4位, 判断是否可以匹配到在test_create中创建的记录
        filters = [
            QueryFilter(op="startswith", field="serial_number", value=number)
        ]
        body = MapBoxQuerySchema(
            page=1,
            page_size=20,
            single=False,
            filters=filters
        )
        resp_raw = client.get(api, json=body.dict())
        resp = resp_raw.json()
        jsonschema.validate(resp, PaginationResponse.schema())
        exits = False
        for d in resp["data"]:
            if d["serial_number"] == self.created_data["serial_number"]:
                exits = True
        assert exits is True, "模糊匹配测试未通过"

    def test_update(self):
        """Test update a record.

        Returns:

        """
        # 先查询一次
        resp_raw = client.get(
            url_prefix + "/",
            json=MapBoxQuerySchema(
                page=1,
                page_size=20,
                single=True,
                filters=[
                    QueryFilter(
                        op="eq",
                        field="serial_number",
                        value=self.created_data["serial_number"],
                    )
                ],
                sort=[],
            ).dict(),
        )
        resp = resp_raw.json()
        old = MapBox.parse_obj(resp["data"])

        # Update
        self.created_data["normal_length"] = 200
        api = url_prefix + "/edit"
        time.sleep(0.5)
        resp_raw = client.post(api, json=self.created_data)
        resp = resp_raw.json()
        new = MapBox.parse_obj(resp["data"])
        assert new.normal_length == 200
        assert new.update_time >= old.update_time

    def test_delete(self):
        """Test delete a record.

        Returns:

        """
        del_api = url_prefix + "/delete"
        body = MapBoxDeleteSchema(
            serial_number=self.created_data["serial_number"]
        )
        resp_raw = client.post(del_api, json=body.dict())
        resp = resp_raw.json()
        try:
            jsonschema.validate(resp, SingleResponse.schema())
        except ValidationError as err:
            print(__name__, resp)
            raise err

        # 删除后, 再做一次查询
        self.test_get_non_existent(number=self.created_data["serial_number"])

    def test_recreate_and_update(self):
        """删除后, 再次重新添加并更新

        Returns:

        """
        # 删除后, 再做一次相同serial_number的添加
        self.create()

        # 添加成功后, 再做一次更新
        self.created_data["normal_length"] = 400
        api = url_prefix + "/edit"
        time.sleep(0.5)
        resp_raw = client.post(api, json=self.created_data)
        resp = resp_raw.json()
        new = MapBox.parse_obj(resp["data"])
        assert new.normal_length == 400

        # 再次删除, 避免影响其他用例
        self.test_delete()

    def test_update_with_invalid_data(self):
        """使用无效数据更新"""
        # 删除后, 再做一次相同serial_number的添加
        self.create()

        # 添加成功后, 再做一次更新
        self.created_data["normal_length"] = 0
        api = url_prefix + "/edit"
        time.sleep(0.5)
        resp_raw = client.post(api, json=self.created_data)
        resp = resp_raw.json()
        assert resp["code"] != 0

        # 再次删除, 避免影响其他用例
        self.test_delete()
