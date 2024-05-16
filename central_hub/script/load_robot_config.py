#!/usr/bin/python
# -*- coding: utf-8 -*-
'''
Copyright (c) XYZ Robotics Inc. - All Rights Reserved
Unauthorized copying of this file, via any medium is strictly prohibited
Proprietary and confidential
Author: Kun Chen<kun.chen@xyzrobotics.ai>, Dec, 2021
'''
import sys
import time
import logging

import rospy
import rosparam

logging.basicConfig(
    level = logging.INFO,
    format = '%(asctime)s %(levelname)s : %(message)s'
)

logger = logging.getLogger(__name__)

def load_rosparam(path):
    """加载ros参数
    """
    params = rosparam.load_file(path)
    for values, ns in params:
        rosparam.upload_params(ns=ns, values=values)

if __name__ == '__main__':

    # path = '/opt/ros/kinetic/bin/rosparam load /home/xyz/xyz_app/app/motion/planner_config.yaml'
    path = '/home/xyz/xyz_app/app/motion/planner_config.yaml'
    if len(sys.argv) > 1:
        path = sys.argv[1]
    
    logger.info("从文件[{}]中加载ros参数".format(path))
    loading = True
    try:
        load_rosparam(path)
    except Exception as e:
        loading = False
        logging.error("参数加载失败", exc_info=True)
    else:
        logger.info("参数加载成功")

    while loading:
        time.sleep(5)