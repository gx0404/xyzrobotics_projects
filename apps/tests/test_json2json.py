import json
import unittest

from apps.utils.json2json import convert
from apps.utils.json2json.piri.process import process
from apps.utils.json2json.build import build_tree


class TestJSONMapping(unittest.TestCase):
    """测试JSON映射转换"""

    def test_json2json1(self):
        data = {
            "taskNo": "123",
            "taskType": "consequat",
            "priority": "1",
            "deviceNo": "111222",
            "uuid": "65",
            "currentTime": "2018-06-04 18:46:30",
            "fromPoint": "0",
            "toPoint": "-",
            "containerNo": "1",
            "skuInfo": {
                "skuNo": "sku no",
                "skuNum": "51",
                "skuType": "eiusmod",
                "weight": 20,
                "length": 19,
                "width": 77,
                "height": 16,
            },
            "fromContainer": "1",
            "toContainer": "1",
        }

        rule = {
            "taskNo": "tasks[].task_id",
            "taskType": "tasks[].task_type",
            "priority": "tasks[].customized_data.priority",
            "deviceNo": "device_id",
            "uuid": "tasks[].customized_data.uuid",
            "currentTime": "tasks[].customized_data.current_time",
            "fromPoint": "tasks[].from.point",
            "fromContainer": "tasks[].from.container",
            "toPoint": "tasks[].to.point",
            "toContainer": "tasks[].to.container",
            "containerNo": "tasks[].customized_data.container_no",
        }

        expected = {
            "device_id": "111222",
            "tasks": [
                {
                    "task_id": "123",
                    "task_type": "consequat",
                    "customized_data": {
                        "priority": "1",
                        "uuid": "65",
                        "current_time": "2018-06-04 18:46:30",
                        "container_no": "1",
                    },
                    "from": {
                        "point": "0",
                        "container": "1",
                    },
                    "to": {
                        "point": "-",
                        "container": "1",
                    },
                }
            ],
        }

        result = convert(data, rule)
        self.assertEqual(result, expected)

    def test_json2json2(self):
        data = {
            "taskNo": "123",
            "stationNo": "1",
            "pickZone": "1",
            "goodsInfo": {
                "goodsName": "带数今快器般",
                "billCode": "76",
                "len": 80,
                "width": 77,
                "height": 100,
                "weight": 10,
            },
            "pickNum": 7,
        }

        rule = {
            "taskNo": "tasks[].task_id",
            "stationNo": "tasks[].customized_data.station_no",
            "pickZone": "tasks[].from.zone",
            "pickNum": "tasks[].target_num",
            "goodsInfo.goodsName": "tasks[].sku_info.sku_name",
            "goodsInfo.billCode": "tasks[].sku_info.sku_id",
            "goodsInfo.len": "tasks[].sku_info.length",
            "goodsInfo.width": "tasks[].sku_info.width",
            "goodsInfo.height": "tasks[].sku_info.height",
            "goodsInfo.weight": "tasks[].sku_info.weight",
        }

        expected = {
            "tasks": [
                {
                    "task_id": "123",
                    "customized_data": {
                        "station_no": "1",
                    },
                    "from": {
                        "zone": "1",
                    },
                    "target_num": 7,
                    "sku_info": {
                        "sku_name": "带数今快器般",
                        "sku_id": "76",
                        "length": 80,
                        "width": 77,
                        "height": 100,
                        "weight": 10,
                    },
                }
            ]
        }

        result = convert(data, rule)
        self.assertEqual(result, expected)

    def test_json2json3(self):
        data = {
            "robotCode": "53",
            "stationCode": 60,
            "taskNo": "cillum pariatur culpa laboris",
            "pickNumber": 72,
            "length": 81,
            "width": 72,
            "height": 70,
            "weight": 70,
            "stationName": "低支军素",
            "commodityName": "际火马天基话",
            "systemCode": "41",
            "parameters": {
                "a": 1,
            },
            "caseNoList": "ullamco",
            "houseCode": "24",
        }

        rule = {
            "robotCode": "device_id",
            "systemCode": "system_code",
            "parameters": "tasks[].customized_data",
            "caseNoList": "tasks[].customized_data.case_no_list",
            "stationCode": "tasks[].customized_data.station_code",
            "stationName": "tasks[].customized_data.station_name",
            "taskNo": "tasks[].task_id",
            "pickNumber": "tasks[].target_num",
            "length": "tasks[].sku_info.length",
            "width": "tasks[].sku_info.width",
            "weight": "tasks[].sku_info.weight",
            "commodityName": "tasks[].sku_info.sku_name",
        }

        expected = {
            "device_id": "53",
            "system_code": "41",
            "tasks": [
                {
                    "task_id": "cillum pariatur culpa laboris",
                    "target_num": 72,
                    "sku_info": {
                        "sku_name": "际火马天基话",
                        "length": 81,
                        "width": 72,
                        "weight": 70,
                    },
                    "customized_data": {
                        "case_no_list": "ullamco",
                        "station_code": 60,
                        "station_name": "低支军素",
                    },
                }
            ],
        }
        
        result = convert(data, rule)
        self.assertEqual(result, expected)

    def test_json2json4(self):
        data = {
            "systemCode": "36",
            "houseCode": "82",
            "deviceCode": "63",
            "robotTasks": [
                {
                    "taskType": "pariatur do dolor aliquip qui",
                    "materialCode": "71",
                    "materialType": "exercitation sit",
                    "materialLength": 39,
                    "materialWidth": 5,
                    "materialHeight": 32,
                    "stackerType": "enim",
                    "channel": 48,
                    "area": 4,
                    "containerCode": "57",
                    "stationCode": "97",
                    "toChannel": 86,
                    "toArea": "officia",
                    "toContainerCode": "81",
                },
                {
                    "taskType": "dolore",
                    "materialCode": "61",
                    "materialType": "consectetur dolore proident",
                    "materialLength": 29,
                    "materialWidth": 25,
                    "materialHeight": 23,
                    "stackerType": "incididunt deserunt nostrud id laborum",
                    "channel": 89,
                    "area": 14,
                    "containerCode": "17",
                    "stationCode": "14",
                    "toChannel": 33,
                    "toArea": "nisi est exercitation",
                    "toContainerCode": "30",
                },
            ],
            "parameters": {},
        }

        rule = {
            "robotTasks[].taskType": "tasks[].task_type",
            "robotTasks[].materialCode": "tasks[].sku_info.sku_id",
            "robotTasks[].materialLength": "tasks[].sku_info.length",
            "robotTasks[].materialWidth": "tasks[].sku_info.width",
            "robotTasks[].materialHeight": "tasks[].sku_info.height",
            "robotTasks[].stackerType": "tasks[].customized_data.stacker_type",
            "robotTasks[].channel": "tasks[].from.channel",
            "robotTasks[].area": "tasks[].from.area",
            "robotTasks[].containerCode": "tasks[].from.container",
            "robotTasks[].stationCode": "tasks[].station_code",
            "robotTasks[].toChannel": "tasks[].to.channel",
            "robotTasks[].toArea": "tasks[].to.area",
            "robotTasks[].toContainerCode": "tasks[].to.container",
            "deviceCode": "device_id",
            "systemCode": "system_code",
            "parameters": "customized_data",
        }

        expected = {
            "device_id": "63",
            "system_code": "36",
            "customized_data": {},
            "tasks": [
                {
                    "task_type": "pariatur do dolor aliquip qui",
                    "sku_info": {
                        "sku_id": "71",
                        "length": 39,
                        "width": 5,
                        "height": 32,
                    },
                    "customized_data": {
                        "stacker_type": "enim",
                    },
                    "from": {
                        "channel": 48,
                        "area": 4,
                        "container": "57",
                    },
                    "station_code": "97",
                    "to": {
                        "channel": 86,
                        "area": "officia",
                        "container": "81",
                    },
                },
                {
                    "task_type": "dolore",
                    "sku_info": {
                        "sku_id": "61",
                        "length": 29,
                        "width": 25,
                        "height": 23,
                    },
                    "customized_data": {
                        "stacker_type": "incididunt deserunt nostrud id laborum",
                    },
                    "from": {
                        "channel": 89,
                        "area": 14,
                        "container": "17",
                    },
                    "station_code": "14",
                    "to": {
                        "channel": 33,
                        "area": "nisi est exercitation",
                        "container": "30",
                    },
                },
            ],
        }

        result = convert(data, rule)
        print(json.dumps(result, indent=4))
        self.assertEqual(result, expected)

    def test_json2json_multi_level(self):
        """Test json2json with multi level."""
        rule = {
            "a.RootTasks[].TaskType": "b.tasks[].task_type",
        }
        data = {
            "a": {
                "RootTasks": [
                    {"TaskType": "1"},
                ]
            }
        }
        config = build_tree(rule)
        # print(json.dumps(config, indent=4))

        expected = {
            "b": {
                "tasks": [
                    {"task_type": "1"},
                ]
            }
        }
        result = process(data, config)
        # print(json.dumps(result, indent=4))
        assert isinstance(result, dict)
        self.assertDictEqual(result, expected)
