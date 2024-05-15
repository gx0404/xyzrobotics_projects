import unittest
import uuid

from flask import url_for

from apps import create_app
from wcs_adaptor import zh_msg
from wcs_adaptor.app import init_app
from wcs_adaptor.entity import SkuInfo
from wcs_adaptor.enums import TaskType
from wcs_adaptor.manager import task_manager, workspace_manager


class TestRafconApi(unittest.TestCase):
    """
    测试Rafcon相关的API.
    """

    def __init__(self, *args, **kwargs):
        from wcs_adaptor.manager import task_manager

        super(TestRafconApi, self).__init__(*args, **kwargs)
        task_manager.__TaskManager_size = 10
        self.app = create_app(testing=True)
        init_app(self.app)
        self.client = self.app.test_client()
        self.socketio_client = self.app.socketio.test_client(
            self.app, flask_test_client=self.client
        )

    def setUp(self) -> None:
        from wcs_adaptor.manager import task_manager

        self.app.globals["rafcon_error"] = None
        self.remove_task()
        task_manager.clear()

    def tearDown(self) -> None:
        from wcs_adaptor.manager import task_manager

        self.app.globals["rafcon_error"] = None
        self.remove_task()
        task_manager.clear()

    def test_report_task_status(self):
        """测试报告任务完成接口."""
        with self.app.app_context(), self.app.test_request_context():
            self.remove_task()
            # 无效输入
            api = url_for("rafcon.report_task_status")
            resp = self.client.post(api, json={})
            assert resp.json["code"] == 10400, resp.json
            message = self.socketio_client.get_received("/")[0]
            assert (
                message["name"] == "order_info"
                and message["args"][0]["msg"] == resp.json["error_message"]
            ), message

            task_id = "123"
            data = {
                "task_id": task_id,
                "pick_num": 1,
                "error": 0,
                "error_message": "",
                "is_depal_pallet_empty": False,
                "is_pal_pallet_full": False,
            }
            # 此次访问无"123"的任务, 应抛出异常
            resp = self.client.post(api, json=data)
            assert resp.json["code"] == 12002, resp.json
            message = self.socketio_client.get_received("/")[0]
            assert (
                message["name"] == "order_info"
                and message["args"][0]["msg"] == resp.json["error_message"]
            ), message

            # 创建一个单拆任务
            def create_task():
                data = {
                    "task_id": task_id,
                    "sku_info": {
                        "sku_id": "123",
                        "length": 1,
                        "width": 1,
                        "height": 1,
                        "weight": 1,
                        "sku_num": 10,
                        "sku_type": "1",
                    },
                    "from": "0",
                    "to": "1",
                }
                self.client.post("/api/wcs/single_class_depal_task", json=data)
                self.socketio_client.get_received("/")

            create_task()
            # 报告此任务(放置托盘非满与抓取托盘不为空)
            data = {
                "task_id": task_id,
                "pick_num": 5,
                "error": 0,
                "error_message": "",
                "is_depal_pallet_empty": False,
                "is_pal_pallet_full": False,
            }
            resp = self.client.post(api, json=data)
            assert resp.json["code"] == 0, resp.json
            messages = self.socketio_client.get_received("/")

            results = []
            for message in messages:
                if message["name"] == "order_info":
                    try:
                        assert f"{data['pick_num']}/10" in message["args"][0]["msg"], message
                        results.append(True)
                    except AssertionError:
                        results.append(False)
            assert len(results) == 1 and len(
                list(
                    filter(
                        lambda x: x,
                        results
                    )
                )
            ) == 1, messages

            assert task_manager.get_task_by_id(task_id)
            task = task_manager.get_task_by_id(task_id)
            assert workspace_manager.get(task.from_ws).is_ready is True
            assert workspace_manager.get(task.to_ws).is_ready is True
            assert data["pick_num"] == task.done_num, "完成数量不一致"

            self.remove_task()

    def test_report_task_status_and_check_task_is_done(self):
        """测试多个任务完成后, 任务是否完成."""
        from wcs_adaptor.depalletize.entity import DPTTask
        from wcs_adaptor.manager import TaskManager

        with self.app.app_context(), self.app.test_request_context():
            tm: TaskManager[DPTTask] = self.app.task_manager

            def add_task(task_id: str):
                task = DPTTask(
                    task_id=task_id,
                    task_type=TaskType.SINGLE_DEPAL,
                    target_num=2,
                    sku_info=SkuInfo(
                        sku_id="123",
                        length=1,
                        width=1,
                        height=1,
                        weight=1,
                    ),
                    from_ws="0",
                    to_ws="1",
                )

                tm.append(task)

            tm.set_capacity(10)
            add_task("123")
            add_task("456")
            api = url_for("rafcon.report_task_status")
            data = {
                "task_id": "123",
                "pick_num": 1,
                "error": 0,
                "error_message": "",
                "is_depal_pallet_empty": False,
                "is_pal_pallet_full": False,
            }
            resp = self.client.post(api, json=data)
            assert resp.json["data"]["is_task_finished"] == False, resp.json
            resp = self.client.post(api, json=data)
            assert resp.json["data"]["is_task_finished"] == True, resp.json

            data["task_id"] = "456"
            resp = self.client.post(api, json=data)
            assert resp.json["data"]["is_task_finished"] == False , resp.json
            tm.clear()
            tm.set_capacity(1)

    def test_get_task_info(self):
        """测试获取任务详情及SKU信息."""
        with self.app.test_request_context():
            self.remove_task()
            # 空任务
            api = url_for("rafcon.get_task_info")
            resp = self.client.post(api)
            assert resp.json["result"] == False, resp.json

            body = {
                "task_id": "1",
                "sku_info": {
                    "sku_id": "123",
                    "length": 1,
                    "width": 1,
                    "height": 1,
                    "weight": 1,
                    "sku_num": 1,
                    "sku_type": "1",
                },
                "from": "0",
                "to": "1",
            }
            self.create_task(task_id="1", data=body)
            resp = self.client.post(api)
            assert resp.json["code"] == 0, resp.json
            self.remove_task()

    def test_is_pick_ws_ready(self):
        """测试获取抓取空间是否就位接口."""
        # 输入异常.
        with self.app.app_context(), self.app.test_request_context():
            self.remove_task()
            resp = self.client.post(url_for("rafcon.is_pick_ws_ready"), json={})
            assert resp.json["code"] == 10400, resp.json

            body = {"task_id": "1", "ws_id": "1"}

            self.create_task(task_id="1")
            resp = self.client.post(url_for("rafcon.is_pick_ws_ready"), json=body)
            assert resp.json["code"] == 0
            self.remove_task()

    def test_is_place_ws_ready(self):
        """测试获取放置空间是否就位接口."""
        # 输入异常.
        with self.app.app_context(), self.app.test_request_context():
            self.remove_task()
            resp = self.client.post(url_for("rafcon.is_place_ws_ready"), json={})
            assert resp.json["code"] == 10400, resp.json

            # 任务不存在异常.
            body = {"task_id": "1", "ws_id": "1"}

            self.create_task(task_id="1")
            resp = self.client.post(url_for("rafcon.is_place_ws_ready"), json=body)
            assert resp.json["code"] == 0
            self.remove_task()

    def test_error_handler(self):
        """测试处理异常接口."""
        with self.app.app_context(), self.app.test_request_context():
            # 输入异常.
            app = create_app()
            api = url_for("rafcon.error_handler")
            resp = self.client.post(api, json={})
            assert resp.json["code"] == 10400, resp.json

            # 输入错误的 code.
            body = {"error": "0", "data": {"error_code": "xxxxxx"}}
            resp = self.client.post(api, json=body)
            assert resp.json == {
                "code": 0,
                "msg": "success",
                "data": {},
                "error": 0,
                "error_message": "success",
            }, resp.json

            # 输入正确的 code.
            body = {"error": "20000", "data": {"error_code": "20000"}}
            resp = self.client.post(api, json=body)
            assert resp.json == {
                "code": 0,
                "msg": "success",
                "data": {},
                "error": 0,
                "error_message": "success",
            }, resp.json

            self.socketio_client.get_received("/")
            # 输入需要回报给WCS的异常.
            body = {
                "error": "E0608",
                "data": {"error_code": "E0608", "error_msg": "this a error message."},
            }
            resp = self.client.post(api, json=body)
            assert resp.json["code"] == 0, resp.json
            messages = self.socketio_client.get_received("/")
            for msg in messages:
                if (
                    msg["name"] == "order_info"
                    and msg["args"][0]["msg"] == zh_msg.ALL_ERROR.get("E0608")["msg"]
                ):
                    break
            else:
                raise AssertionError("消息错误.")

            # rafcon_error = app.globals.get("rafcon_error")
            # assert rafcon_error["error_code"] == "E0608"
            # assert rafcon_error["error_msg"] == "this a error message."

            # 还原全局变量
            app.globals["rafcon_error"] = None

    def test_get_sku_info(self):
        """测试获取 sku 信息."""
        with self.app.app_context(), self.app.test_request_context():
            api = url_for("rafcon.get_sku_info")
            # 输入异常.
            resp = self.client.post(api, json={})
            assert resp.json["code"] == 10400

            # 输入sku
            # TODO(YuhangWu): 涉及 wcsserver 的模拟
            #   等待 send_request 重构后再补全此测试用例.
            # body = {
            #     "sku_id": "1",
            #     "customized_request": {}
            # }
            # self.client.post(api, json=body)
            # assert resp.json == {
            #
            # }

    def test_get_error(self):
        """测试获取异常信息"""
        with self.app.test_request_context():
            # 创建异常前, 获取异常信息
            resp = self.client.get(url_for("rafcon.get_error"))
            assert (
                resp.json["data"]["error_data"] == None
                and resp.json["data"]["rafcon_error"] == False
            )

            # 创建异常
            api = url_for("rafcon.error_handler")
            error_data = {
                "error": "E0000",
                "data": {
                    "error_msg": "Start task failure",
                    "zh_msg": "任务开始失败",
                    "error_code": "E0000",
                    "tip": "请联系XYZ开发人员",
                },
            }
            resp = self.client.post(api, json=error_data)

            assert resp.json["code"] == 0

            # 获取异常
            resp = self.client.get(url_for("rafcon.get_error"))
            assert (
                resp.json["data"]["error_data"]["code"] == error_data["error"]
            ), resp.json

            # 获取一次异常后, app内rafcon异常就会被重置为None
            # 在此处检查app的rafcon异常信息是否为None
            resp = self.client.get(url_for("rafcon.get_error"))
            assert (
                resp.json["data"]["error_data"] == None
                and resp.json["data"]["rafcon_error"] == False
            )

    def remove_task(self):
        """清空任务."""
        resp = self.client.post("/api/cmd/remove_task")
        self.socketio_client.get_received("/")
        assert resp.json == {
            "code": 0,
            "data": {},
            "error": 0,
            "error_message": "",
            "msg": "",
        }, resp

    def create_task(
        self, task_id: str, path: str = "/single_class_depal_task", data: dict = None
    ):
        """创建任务

        Args:
            task_id(str): 任务ID.
            path(str): 各个任务类型的路由. Default: "/single_class_depal_task".
            data(dict): 请求体. Default:
                {
                    "task_id": task_id,
                    "sku_info": {
                        "sku_id": "123",
                        "length": 1,
                        "width": 1,
                        "height": 1,
                        "weight": 1,
                        "sku_num": 1,
                        "sku_type": "1",
                    },
                    "from": "0",
                    "to": "1",
                }

        Returns:
            resp: 响应对象

        """
        if data is None:
            data = {
                "task_id": task_id,
                "sku_info": {
                    "sku_id": "123",
                    "length": 1,
                    "width": 1,
                    "height": 1,
                    "weight": 1,
                    "sku_num": 1,
                    "sku_type": "1",
                },
                "from": "0",
                "to": "1",
            }
        resp = self.client.post("/api/wcs" + path, json=data)
        return resp

    def test_report_task_ending(self):
        """测试上报任务结束."""
        from wcs_adaptor.depalletize.entity import DPTTask
        from wcs_adaptor.manager import TaskManager

        with self.app.app_context(), self.app.test_request_context():
            tm: TaskManager[DPTTask] = self.app.task_manager

            def add_task(_id: str):
                task = DPTTask(
                    task_id=_id,
                    task_type=TaskType.SINGLE_DEPAL,
                    target_num=2,
                    sku_info=SkuInfo(
                        sku_id="123",
                        length=1,
                        width=1,
                        height=1,
                        weight=1,
                    ),
                    from_ws="0",
                    to_ws="1",
                )

                tm.append(task)

            task_id = uuid.uuid4().__str__()
            add_task(task_id)
            data = {
                "task_id": task_id,
                "pick_num": 1,
                "error": 0,
                "error_message": "",
                "is_depal_pallet_empty": False,
                "is_pal_pallet_full": False,
            }
            resp = self.client.post(url_for("rafcon.report_task_status"), json=data)
            assert resp.json["data"]["is_task_finished"] == False, resp.json
            resp = self.client.post(url_for("rafcon.report_task_status"), json=data)
            assert resp.json["data"]["is_task_finished"] == True, resp.json

            # 检查任务状态为已完成, get_task_info
            resp = self.client.post(url_for("rafcon.get_task_info"))
            assert resp.json["data"]["result"] == True, resp.json
            # 2 表示任务已完成
            assert resp.json["data"]["task_status"] == 2, resp.json

            # 报告任务已结束, report_task_ending
            resp = self.client.post(url_for("rafcon.report_task_ending"), json={"task_id": task_id})
            assert resp.json["code"] == 0, resp.json

            # 检查队列中是否有该任务, /api/manager/task/<task_id>
            assert task_manager.first() == None, "队列中仍有任务"

            # 检测历史任务表中该记录是否存在, /api/task_history
            resp = self.client.post(url_for("task_history.list_tasks"), json={
                "filters": [
                    {
                        "op": "eq",
                        "field": "task_id",
                        "value": task_id
                    }
                ]
            })
            assert resp.json["count"] == 1, resp.json
