# -*- coding: utf-8 -*-

"""
Copyright (c) XYZ Robotics Inc. - All Rights Reserved
Unauthorized copying of this file, via any medium is strictly prohibited
Proprietary and confidential
Author: Michael Su <michael.su@xyzrobotics.ai>, 4/3/2022
"""

import unittest
import requests
import os 
import zipfile


class TestNodeApi(unittest.TestCase):

    def setUp(self):
        self.url_download_node_log = "http://localhost:7002/api/download/log"
        self.url_get_node_log = "http://localhost:7002/api/query/node_log"

    def test_download_log(self):
        data = {
            "node_name": "node_state_listener",
        }
        response = requests.post(self.url_download_node_log, json=data)
        attachment = response.headers.get("content-disposition")[1]
        filename = attachment.split("=", 1)[-1]
        with open(filename, 'wb') as zipFile:
            zipFile.write(response.content)
        assert zipfile.is_zipfile(filename)
        os.remove(filename)


    def test_get_node_log(self):
        data = {
            "node_name": "test_node",
            "num_of_log": 200
        }
        response = requests.post(self.url_get_node_log, json=data)

if __name__ == '__main__':
    unittest.main()