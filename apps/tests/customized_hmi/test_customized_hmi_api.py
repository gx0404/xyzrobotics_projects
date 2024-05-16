# -*- coding: utf-8 -*-

"""
Copyright (c) XYZ Robotics Inc. - All Rights Reserved
Unauthorized copying of this file, via any medium is strictly prohibited
Proprietary and confidential
Author: Michael Su <michael.su@xyzrobotics.ai>, June 3 2022
"""

import unittest
import requests
import os 
import zipfile
import json

class TestCustomizedHmiApi(unittest.TestCase):

    def setUp(self):
        self.url_customized_hmi_route = "http://localhost:7002/api/hmi/customized_hmi"

    def test_customized_hmi_api(self):
        with open ("/home/xyz/xyz_app/app/xyz_logistics_hmi_back/config/default.json", "r") as fin:
            default_cg = json.load(fin)
        with open(default_cg["custom_hmi_cg"], "r") as fin:
            custom_hmi_cg = json.load(fin)
        assert requests.post(self.url_customized_hmi_route).json()["data"] == custom_hmi_cg

if __name__ == '__main__':
    unittest.main()