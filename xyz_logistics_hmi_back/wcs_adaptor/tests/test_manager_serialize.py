import time
import uuid
from datetime import datetime
import unittest

import pandas as pd

from apps import create_app
import wcs_adaptor

app = create_app()
wcs_adaptor.init_app(app)
client = app.test_client()
times = 10000


def create_task(task_id):
    """Create a task.

    Args:
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
        "from_ws": "0",
        "to_ws": "1"
    }
    client.post("/api/wcs/single_class_depal_task", json=body)


class TestManagerSerialize(unittest.TestCase):

    def test_create(self):
        data = {
            "datetime": [],
            "time": [],
            "times": []
        }

        for i in range(times):
            uid = uuid.uuid4().__str__()
            before = time.time()
            create_task(task_id=uid)
            after = time.time()
            data["datetime"].append(
                datetime.now().__str__()
            )
            data["time"].append(after - before)
            data["times"].append(i + 1)

        # df = pd.DataFrame(data, index=[0])
        df = pd.DataFrame(data)
        df.to_csv("1.csv")
