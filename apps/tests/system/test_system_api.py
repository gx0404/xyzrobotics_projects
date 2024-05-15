"""
测试系统接口
"""
import base64
import json
import os
import unittest
import jsonschema

from werkzeug.datastructures import FileStorage

from apps import create_app
from apps.schemas import Response
from apps.base_app.views.system.schemas import SystemInfoResponse, LogoResponse

app = create_app()
client = app.test_client()
FILE_PATH = os.path.abspath(__file__)


class TestSystemApi(unittest.TestCase):
    """测试系统API"""

    def test_get_system_info(self):
        """测试获取系统信息."""
        resp = client.get("/api/system/")
        json = resp.json
        jsonschema.validate(json, SystemInfoResponse.schema())

    def test_edit_log(self):
        """测试编辑logo."""
        # 1. 上传文件.
        path = os.path.abspath(
            os.path.join(os.path.dirname(FILE_PATH), "logo.png"))
        file = FileStorage(
            stream=open(path, "rb"),
            filename="logo.png",
        )
        data = {"file": file}
        resp = client.post("/api/system/logo/", data=data)
        jsonschema.validate(resp.json, LogoResponse.schema())
        # 2. 获取文件后比对
        logo_resp = LogoResponse.parse_obj(resp.json)
        b1 = base64.b64decode(logo_resp.data.logo)
        with open(path, "rb") as f:
            b = f.read()
        assert b == b1

    def test_get_logo(self):
        """测试获取logo."""
        path = os.path.abspath(
            os.path.join(os.path.dirname(FILE_PATH), "logo.png"))
        with open(path, "rb") as f:
            b = f.read()
        resp = client.get("/api/system/logo/")
        jsonschema.validate(resp.json, LogoResponse.schema())
        logo_resp = LogoResponse.parse_obj(resp.json)
        b1 = base64.b64decode(logo_resp.data.logo)
        assert b == b1

    def test_edit_language(self):
        """测试修改系统语言."""
        data = {"language": 2}
        resp = client.post("/api/system/lang/", json=data)
        print(resp.json)
        json_data = json.loads(resp.data.decode())
        jsonschema.validate(json_data, Response.schema())
        result = Response.parse_obj(json_data)
        assert result.code == 0

        # 传入错误的code
        data = {"language": 10000}
        resp = client.post("/api/system/lang/", json=data)
        assert resp.json["code"] == 400 and resp.json["data"]["error_code"] == 10400, resp.json
