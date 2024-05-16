# -*- coding: utf-8 -*-
"""
Copyright (c) XYZ Robotics Inc. - All Rights Reserved
Unauthorized copying of this file, via any medium is strictly prohibited
Proprietary and confidential
Author: Michael Su <michael.su@xyzrobotics.ai>, 4/3/2022
"""

import unittest
import json
import requests
import os 
import sys


# Enable to import utils
FILE_PATH = os.path.abspath(__file__)
sys.path.append(os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(FILE_PATH))))))
print(sys.path)

class TestLocalHelperPipeline(unittest.TestCase):

    def setUp(self):
        self.url_req_label = "http://localhost:7002/api/helper/request_label/"
        self.url_req_finish_label = "http://localhost:7002/api/helper/finish_label"

    def test_local_helper_pipeline(self):
        from utils.remote_helper.box_label_remote_helper import BoxLabelRemoteHelper
        task_id = None
        help_dict = {
            "image_path": os.path.abspath(os.path.join(os.path.dirname(FILE_PATH), "box1.png")),
            "camera_id": "1",
            "annotation_json_string": json.dumps({"roi":[[302.0,148.0],[956.0,148.0],[956.0,707.0],[302.0,707.0]],"height":960,"width":1280,"instances":[],"segments":[],"image_category":"","algorithm":"suction_based","pred_instances":[{"instance_category":"sku","score":0.9998699426651001,"bbox":[401.0,226.0,176.0,219.0],"id":"sLmg8zUl","segmentation":[[[558.0,226.0],[401.0,235.0],[416.0,444.0],[576.0,435.0]]],"parts":[{"name":"suction","id":"dYOqUFCE","polygon":[[416.0,446.0],[401.0,237.0],[561.0,225.0],[576.0,434.0]]}]},{"instance_category":"sku","score":0.9997056126594544,"bbox":[565.0,216.0,169.0,220.0],"id":"iwgohFey","segmentation":[[[721.0,216.0],[566.0,225.0],[580.0,435.0],[733.0,427.0]]],"parts":[{"name":"suction","id":"PldiuLI4","polygon":[[580.0,436.0],[567.0,225.0],[720.0,216.0],[733.0,427.0]]}]},{"instance_category":"sku","score":0.9994638562202454,"bbox":[416.0,439.0,167.0,213.0],"id":"XuRw46QE","segmentation":[[[570.0,439.0],[416.0,450.0],[430.0,651.0],[582.0,644.0]]],"parts":[{"name":"suction","id":"3ldVzfPr","polygon":[[429.0,654.0],[416.0,449.0],[569.0,439.0],[582.0,644.0]]}]},{"instance_category":"sku","score":0.9992002844810486,"bbox":[724.0,202.0,170.0,219.0],"id":"FcPBTC9x","segmentation":[[[878.0,202.0],[725.0,212.0],[740.0,420.0],[893.0,411.0]]],"parts":[{"name":"suction","id":"4eVfny37","polygon":[[740.0,422.0],[725.0,213.0],[878.0,202.0],[893.0,411.0]]}]},{"instance_category":"sku","score":0.9990203380584717,"bbox":[736.0,426.0,195.0,242.0],"id":"oOamfBip","segmentation":[[[925.0,426.0],[737.0,438.0],[755.0,667.0],[930.0,661.0]]],"parts":[{"name":"suction","id":"3sHIaNpK","polygon":[[759.0,668.0],[747.0,435.0],[923.0,426.0],[935.0,659.0]]}]}]})
        }

        # test rquest_helper_from_server function 
        b = BoxLabelRemoteHelper()
        task_id = b.request_help_from_server(help_dict, msg="hello")
        assert task_id is not None

        # test get label task (hmi)
        response = requests.get(self.url_req_label)
        assert response.json()["data"]["task_id"] is not None

        # 二次获取接口数据，检查数据是否能二次获取
        response = requests.get(self.url_req_label)
        assert response.json()["data"]["task_id"] is not None

        # test finish label task (hmi)
        data = {
            "task_id": task_id,
            "annotation_json_string": "{}",
            "error_code": 0,
            "error_msg": "success"
        }
        response = requests.post(self.url_req_finish_label, data=data)
        assert response.json()['code'] == 0

        # test request_result_from_server function 
        res = b.request_result_from_server(task_id)
        assert res['error_code'] == 0
        result = res['result']
        assert result['annotation_json_string']


if __name__ == '__main__':
    unittest.main()