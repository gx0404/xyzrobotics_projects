import unittest

from apps import create_app
from apps.exceptions import XYZBaseError

app = create_app(testing=True)
client = app.test_client()


@app.route("/generic_error", methods=["GET"])
def generic_error():
    raise Exception("generic error.")


@app.route("/xyz_error", methods=["GET"])
def xyz_error():
    raise XYZBaseError(error_message="xyz error.")


class TestErrorHandler(unittest.TestCase):

    def test_handler_generic_error(self):
        """测试通用异常的处理."""
        resp = client.get("/generic_error")
        assert resp.json["code"] == 500, resp.json

    def test_handler_xyz_error(self):
        resp = client.get("/xyz_error")
        assert resp.json == {
            "code": -1,
            "msg": "XYZ Error",
            "data": {
                "error_code": 10000,
                "error_message": "xyz error."
            }
        }, resp.json
