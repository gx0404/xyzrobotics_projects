#!/usr/bin/env python
# -*- coding:utf-8 -*-
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
from supervisor import childutils
from xyz_central_hub.client import NodeClient

logging.basicConfig(
    level = logging.INFO,
    format = '%(asctime)s %(levelname)s : %(message)s'
)
logger = logging

PY2 = True if sys.version_info[0] == 2 else False


def get_now_timestamp():
    return int(time.time() * 1000)

def json_formatter(json_data, indent=2):
    if PY2:
        stream = io.BytesIO()
    else:
        stream = io.StringIO()
    json.dump(json_data, stream, indent=indent)
    stream.seek(0)
    return stream.read()

def request_formatter(headers, body):
    result = {
        "from_state": body["from_state"], 
        "to_state": headers["eventname"][14:], 
        "timestamp": headers["timestamp"], 
        "group_name": body["groupname"], 
        "program_name": body["processname"],
        "serial": headers["serial"]
    }
    return result

def write_stdout(msg):
    # only eventlistener protocol messages may be sent to stdout
    sys.stdout.write(msg)
    sys.stdout.flush()
 
def write_stderr(msg):
    sys.stderr.write(msg)
    sys.stderr.flush()

def read_stdin():
    return sys.stdin.readline()

def main():
    node_client = NodeClient()
    node_client.connect_sock()

    while 1:
        headers, payload = childutils.listener.wait()
        headers["timestamp"] = get_now_timestamp()
        logger.info("headers > {}".format(headers))
        logger.info("payload > {}".format(payload))

        if headers['eventname'].startswith('TICK'):
            # do nothing with TICK events
            childutils.listener.ok(sys.stdout)
            continue

        body = dict([x.split(":", 1) for x in payload.split()])
        params = request_formatter(headers, body)
        logger.info("there is a node state changed > \n{}".format(params))
        node_client.send(params)

        childutils.listener.ok(sys.stdout)


if __name__ == '__main__':
    main()