# -*- coding: utf-8 -*-
"""
Copyright (c) XYZ Robotics Inc. - All Rights Reserved
Unauthorized copying of this file, via any medium is strictly prohibited
Proprietary and confidential
Author: Kun Chen <kun.chen@xyzrobotics.ai>, 2022-11-05
"""

import os
import time
import datetime
import unittest
from shutil import rmtree

from flask import url_for

from apps import settings, create_app
import wcs_adaptor


class TestLowcodeApi(unittest.TestCase):
    """测试自定义功能接口
    """

    def setUp(self) -> None:
        self.app = create_app(testing=True)
        self.client = self.app.test_client()
        
        wcs_adaptor.init_app(self.app)
        
        self.base_log_path = "/home/xyz/xyz_log"

    def clear_vision_images(self, src: str):
        """清除视觉图片的源文件夹和软链接
        
        Args:
            src: str, 视觉图片目录路径
        """
        if os.path.exists(src):
            rmtree(src)

        link_pathname = "temp_vision_images"
        dst = f"{settings.STATIC_FOLDER}/{link_pathname}"
        if os.path.islink(dst):
            os.unlink(dst)

    def create_fake_images(self, path: str, nums: int = 5) -> bool:
        """创建假的视觉图像（文件以文本形式）

        Args:
            path: str, 图像文件路径
            nums: int, 需要创建的文件数量
        """
        if not os.path.exists(path):
            os.makedirs(path)
        # 创建图片名称的文本文件
        timestamp = int(time.time() * 1000)
        for i in range(1, nums+1):
            filename = f"{timestamp}_207000127436_{i}_viz.jpg"
            with open(f"{path}/{filename}", "w") as fw:
                pass

    def test_get_latest_pictures(self):
        """测试获取最新的视觉图像"""
        date_format = "test_%Y%m%d"
        folder_name = datetime.date.today().strftime(f"{date_format}_offline")
        vision_folder = f"{self.base_log_path}/{folder_name}"
        image_path = f"{vision_folder}/debug"
        image_nums = 5
        with self.app.app_context(), self.app.test_request_context():
            api = url_for("lowcode.get_latest_pictures")
        # 清空之前的测试图片
        self.clear_vision_images(src=vision_folder)
        # 创建测试的图片
        self.create_fake_images(path=image_path, nums=image_nums)
        # 获取图片列表
        response = self.client.post(
            api,
            json={
                "host": "localhost",
                "port": 7002,
                "date_format": date_format
            }
        )
        print('[+] lowcode.get_latest_pictures的响应结果1', response.json)
        image_list = response.json["data"]["image_list"]
        assert len(image_list) == image_nums, response.json

        # 清空之前的测试图片
        self.clear_vision_images(src=vision_folder)
        # 获取图片列表
        response = self.client.post(
            api,
            json={
                "host": "localhost",
                "port": 7002,
                "date_format": date_format
            }
        )
        print('[+] lowcode.get_latest_pictures的响应结果2', response.json)
        image_list = response.json["data"]["image_list"]
        assert len(image_list) == 0, response.json

        # 清空之前的测试图片
        self.clear_vision_images(src=vision_folder)
    
    def test_get_robot_status(self):
        """测试获取当前机器人的运行状态接口"""
        with self.app.app_context(), self.app.test_request_context():
            # 获取当前机器人的运行状态
            resp = self.client.get(url_for("lowcode.get_robot_status"))
            assert resp.json["data"]["robot_status"] in ["停止","运行","暂停","故障"]

    def test_get_control_mode(self):
        """测试获取当前控制模式接口"""
        with self.app.app_context(), self.app.test_request_context():
            # 获取当前控制模式
            resp = self.client.get(url_for("lowcode.get_control_mode"))
            assert resp.json["data"]["control_mode"] in ["袋型","就地","远程"]

    def test_get_blanking_bags(self):
        """测试获取已下料袋数接口"""
        with self.app.app_context(), self.app.test_request_context():
            # 获取已下料袋数
            resp = self.client.get(url_for("lowcode.get_blanking_bags"))
            assert resp.json["code"] == 0, resp.json

    def test_set_blanking_bags(self):
        """测试设置下发袋数接口"""
        with self.app.app_context(), self.app.test_request_context():
            # 获取设置的下发袋数
            resp = self.client.post(
                url_for("lowcode.set_blanking_bags"),
                json={
                    "bags": 123
                }
            )  
            assert resp.json["data"]["bags"] == 123
