#!/usr/bin/env python
# -*- coding:utf-8 -*-
'''
Copyright (c) XYZ Robotics Inc. - All Rights Reserved
Unauthorized copying of this file, via any medium is strictly prohibited
Proprietary and confidential
Author: Ao Xiao<ao.xiao@xyzrobotics.ai>, August, 2021
Author: Kun Chen<kun.chen@xyzrobotics.ai>, August, 2021
'''
import os
import sys
import time
# import fcntl
import signal
import logging
import subprocess

logging.basicConfig(
    level = logging.INFO,
    format = '%(asctime)s %(levelname)s : %(message)s'
)

start_nodes = ["/usr/bin/xvf_dockerd",
               "/usr/bin/xvf_docker"]
# stop_nodes = ["docker stop phoxi-container",
#               "docker stop xyz-vision-flow-container"]
stop_nodes = ["docker stop xyz-vision-flow-container"]

if "--no-gui" in sys.argv:
    start_node = start_nodes[0]
else:
    start_node = start_nodes[1]


def quit(signum, frame):
    logging.warning(">>> signum: %s, frame: %s" %(signum, frame))
    logging.info(" close vision flow ".center(80, "*"))

    for node in stop_nodes:
        p = subprocess.Popen(args=node.split())
        p.wait()


logging.info(' start vision flow '.center(80, "*"))

# 如果之前有开启的vision flow，那么先关闭之前已开启过vision flow
for node in stop_nodes:
    p = subprocess.Popen(args=node.split())
    p.wait()
# 启动vision flow
pro = subprocess.Popen(args=start_node.split())

# 设置signal，以确保vision flow能安全退出
signal.signal(signal.SIGTERM, quit)
signal.signal(signal.SIGINT, quit)

# 阻塞，等待vision flow退出信号
logging.info(' vision flow is running'.center(80, "-"))
pro.wait()
