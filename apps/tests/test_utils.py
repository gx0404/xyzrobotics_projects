# -*- coding: utf-8 -*-
"""
Copyright (c) XYZ Robotics Inc. - All Rights Reserved
Unauthorized copying of this file, via any medium is strictly prohibited
Proprietary and confidential
Author: Yuhang Wu <yuhang.wu@xyzrobotics.ai>, 2022-06-23

工具函数的测试用例。
"""
import unittest

from marshmallow import Schema, fields
from pydantic import BaseModel

from apps import make_json_response, validator, create_app
from apps.utils.pydantic_from_marshmallow import pydantic_from_marshmallow

app = create_app(testing=True)


class SubSchemaByMarsh(Schema):
    a = fields.Integer()


class TestSchemaByMarsh(Schema):
    sub = fields.Nested(SubSchemaByMarsh)


class SubSchemaByPyd(BaseModel):
    a: int


class TestSchemaByPyd(BaseModel):
    sub: SubSchemaByPyd


@app.route("/utils/test_validator1", methods=["POST"])
@validator()
def main1(body: TestSchemaByMarsh):
    return make_json_response(data=body.sub.dict())  # type: ignore


@app.route("/utils/test_validator2", methods=["POST"])
@validator()
def main2(body: TestSchemaByPyd):
    return make_json_response(data=body.sub.dict())


class TestUtils(unittest.TestCase):

    def test_validator(self):
        client = app.test_client()
        with app.test_request_context():
            resp = client.post("/utils/test_validator1", json={"sub": {"a": 1}})
            assert resp.json["data"] == {"a": 1}, resp.json

            resp = client.post("/utils/test_validator2", json={"sub": {"a": 2}})
            assert resp.json["data"] == {"a": 2}, resp.json

    def test_pydantic_from_marshmallow(self):
        model = pydantic_from_marshmallow(TestSchemaByMarsh)
        body = model.parse_obj({"sub": {"a": 1}})
        assert body.sub.a == 1
