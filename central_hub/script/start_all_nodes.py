#!/usr/bin/python
# -*- coding: utf-8 -*-
'''
Copyright (c) XYZ Robotics Inc. - All Rights Reserved
Unauthorized copying of this file, via any medium is strictly prohibited
Proprietary and confidential
Author: Kun Chen<kun.chen@xyzrobotics.ai>, August, 2021
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

logging.info("start all of nodes")
time.sleep(2)
hub = HubClient()
result = hub.start_all_nodes()

if PY2:
    stream = io.BytesIO()
else:
    stream = io.StringIO()
json.dump(result, stream, indent=2)
stream.seek(0)


logging.info(stream.read())
logging.info(" quit ".center(20, "*"))