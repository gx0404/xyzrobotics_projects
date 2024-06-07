import unittest

from flask import url_for, current_app

from apps import create_app
import wcs_adaptor
from wcs_adaptor.manager import task_manager
from wcs_adaptor.depalletize.entity import DPTTask
from wcs_adaptor.enums import TaskType


class TestGrating(unittest.TestCase):
    """测试光栅控制接口"""

    def setUp(self) -> None:
        self.app = create_app(testing=True)
        self.client = self.app.test_client()
        wcs_adaptor.init_app(self.app)
        task_manager.clear()

    def tearDown(self) -> None:
        task_manager.clear()

    def test_enable_grating_if_task_manager_not_empty(self):
        """测试开启光栅, 当 manager 中含有任务时"""
        with self.app.app_context(), self.app.test_request_context():
            task = DPTTask(
                task_id="123",
                target_num=10,
                task_type=TaskType.SINGLE_DEPAL,
                from_ws="1",
                to_ws="0",
            )
            task_manager.append(task)
            resp = self.client.post(url_for("grating.enable"))
        assert resp.json["code"] == 12015  # type: ignore

    def test_disable_grating_if_task_manager_not_empty(self):
        """测试禁用光栅, 当 manager 中含有任务时"""
        with self.app.app_context(), self.app.test_request_context():
            task = DPTTask(
                task_id="123",
                target_num=10,
                task_type=TaskType.SINGLE_DEPAL,
                from_ws="1",
                to_ws="0",
            )
            task_manager.append(task)
            resp = self.client.post(url_for("grating.disable"))
        assert resp.json["code"] == 12015  # type: ignore

    # WARN: 无法模拟 PLC 写入信号，以下用例暂时取消

    # def test_enable_grating(self):
    #     """测试启用光栅"""
    #     with self.app.app_context(), self.app.test_request_context():
    #         resp = self.client.post(url_for("grating.enable"))
    #     assert resp.json["code"] == 0, resp.json  # type: ignore,

    # def test_disable_grating(self):
    #     """测试禁用光栅"""
    #     with self.app.app_context(), self.app.test_request_context():
    #         resp = self.client.post(url_for("grating.disable"))
    #     assert resp.json["code"] == 0, resp.json  # type: ignore

    # def test_enable_grating_if_system_status_is_ready(self):
    #     """当系统状态处于就绪状态时，测试开启光栅"""
    #     with self.app.app_context(), self.app.test_request_context():
    #         current_app.status = "ready"
    #         resp = self.client.post(url_for("grating.enable"))
    #     assert resp.json["code"] == 0, resp.json  # type: ignore

    # def test_disable_grating_if_system_status_is_ready(self):
    #     """当系统状态处于就绪状态时，测试开启光栅"""
    #     with self.app.app_context(), self.app.test_request_context():
    #         current_app.status = "ready"
    #         resp = self.client.post(url_for("grating.disable"))
    #     assert resp.json["code"] == 0, resp.json  # type: ignore
