import unittest

import requests


class TestWCSTask(unittest.TestCase):
    """
    Test task api of WCS.
    """

    def setUp(self):
        self.client = requests.session()
        self.host = "http://localhost:7002"

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
                "sku_num": 3
            },
            "from": "0",
            "to": "1"
        }
        resp_raw = self.client.post(
            f"{self.host}/api/wcs/{task_type}",
            json=body
        )
        resp = resp_raw.json()
        assert resp["code"] == 0, resp
        return resp

    def remove_all_tasks(self):
        """Remove all tasks.
        
        Returns:
            a response.

        """
        resp_raw = self.client.post(f"{self.host}/api/cmd/remove_task")
        resp = resp_raw.json()
        return resp

    def test_create_task(self):
        """Test create tasks.
        
        Returns:

        """
        self.remove_all_tasks()
        self.create_task("single_class_depal_task", "122222")

    def test_remove_all_task(self):
        """Test remove all task.
        
        Returns:

        """
        self.remove_all_tasks()
        task_id = "123"
        # Create a task.
        resp = self.create_task("single_class_depal_task", task_id)
        print("Created task: ", resp)

        # Remove task.
        # TODO: 没有移除单个任务的接口, 只有全部移除
        resp = self.remove_all_tasks()
        print("Removed task:", resp)
        success = {
            "code": 0,
            "data": {},
            "error": 0,
            "error_message": '',
            "msg": ''
        }
        assert resp == success

        # 当任务为空时, 再测试一次
        resp = self.remove_all_tasks()
        assert resp == success

    def test_get_unfinished_task_from_not_empty_tasks(self):
        """在有任务的情况下, 查看未完成的任务
        
        Returns:

        """
        self.create_task("single_class_depal_task", "11111111")
        url = "/api/query/has_unfinished_task"
        resp_raw = self.client.post(self.host + url)
        resp = resp_raw.json()
        print("response: ", resp)
        success = {"code": 0, "data": 1, "msg": "success"}
        assert resp == success

    def test_get_unfinished_task_from_empty_tasks(self):
        """在无任务的状态下午, 查看未完成的任务
        
        Returns:

        """
        url = "/api/query/has_unfinished_task"
        self.remove_all_tasks()
        resp_raw = self.client.post(self.host + url)
        resp = resp_raw.json()
        print("response: ", resp)
        success = {"code": 0, "data": 0, "msg": "success"}
        assert resp == success

    def test_get_tasks_from_manager(self):
        """测试从任务管理器中获取多个任务."""
        self.remove_all_tasks()
        api = self.host + "/api/manager/task"
        # 获取空任务列表.
        resp = self.client.get(api)
        assert resp.json()["data"] == [], resp.json()

        # 创建任务
        self.create_task("single_class_depal_task", "123")
        # 获取有任务列表
        resp = self.client.get(api)
        data = resp.json()
        assert data["data"][0]["task_id"] == "123", data
        self.remove_all_tasks()

    def test_get_one_task_from_manager(self):
        """测试从任务管理器中获取指定任务ID的任务."""
        self.remove_all_tasks()
        api = self.host + "/api/manager/task"
        # 获取空任务列表
        resp = self.client.get(api)
        print(resp.text)
        assert resp.json()["data"] == [], resp.json()

        # 创建任务
        self.create_task("single_class_depal_task", "123")
        # 获取有任务列表
        resp = self.client.get(api + "/123")
        data = resp.json()
        assert isinstance(data["data"], dict) and data["data"][
            "task_id"] == "123", data
        self.remove_all_tasks()
