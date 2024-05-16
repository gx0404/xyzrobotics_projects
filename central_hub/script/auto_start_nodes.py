#!/usr/bin/python
# -*- coding: utf-8 -*-
'''
Copyright (c) XYZ Robotics Inc. - All Rights Reserved
Unauthorized copying of this file, via any medium is strictly prohibited
Proprietary and confidential
Author: Kun Chen<kun.chen@xyzrobotics.ai>, March, 2022
'''

import io
import sys
import time
import json
import logging
from xyz_central_hub.client import HubClient

PY2 = True if sys.version_info[0] == 2 else False

logging.basicConfig(
    level = logging.INFO,
    format = '%(asctime)s %(levelname)s : %(message)s'
)

boot_sequence = [
    {
        "node_id": "data_uploader",
        "load_time": 3,
    },
]

def start(node_id, load_time=0):
    logging.info("按顺序启动节点【{}】".format(node_id))
    if load_time > 0:
        logging.info("等待{}秒...".format(load_time))
        time.sleep(load_time)
    hub = HubClient()
    result = hub.start_node(node_id)
    if PY2:
        stream = io.BytesIO()
    else:
        stream = io.StringIO()
    json.dump(result, stream, indent=2)
    stream.seek(0)
    logging.info(stream.read())
    logging.info("节点【{}】启动完成！".format(node_id))

def main():
    logging.info(" 程序启动 ".center(20, "*"))

    for item in boot_sequence:
        node_id = item["node_id"]
        load_time = item["load_time"]
        start(node_id=node_id, load_time=load_time)

    while True:
        time.sleep(60)

    logging.info(" 程序退出 ".center(20, "*"))


if __name__ == '__main__':
    main()
