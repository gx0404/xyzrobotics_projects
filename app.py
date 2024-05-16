# -*- coding: utf-8 -*-

from xyz_motion import RobotDriver
from xyz_motion import SE3
from xyz_vision_lib.xyz_vision_bridge import XYZVisionBridge
from google.protobuf.json_format import MessageToDict
import time,json

scan_pose_list = [
[0.11431871654689735,
 1.086038851344363,
 1.5109698488597512,
 -0.7026609912578318,
 0.7115247932184524,
 0.0,
 0.0],
 [-0.6012709756547898,
 1.080209194660238,
 1.5109698703162493,
 -0.7026609850486061,
 0.7115247993503264,
 0.0,
 0.0],
 [-0.6002927080844811,
 1.4750722196608974,
 1.5109698882210278,
 -0.7026609726301556,
 0.7115248116140742,
 0.0,
 0.0],
 [0.11431871654689735,
 1.4750722196608974,
 1.5109698882210278,
 -0.7026609726301556,
 0.7115248116140742,
 0.0,
 0.0]
]
vision_bridge = XYZVisionBridge()

r = RobotDriver(0)
for scan_pose in scan_pose_list:
    r.set_cart_movel(SE3(scan_pose))
    time.sleep(1)
    ftr = vision_bridge.async_run(int(0), "capture_images",info=json.dumps({"tf_world_hand":scan_pose}))
    time.sleep(2)
    try:
        capture_res = ftr.get(int(0))
        capture_res = MessageToDict(capture_res)
        print(capture_res)
    except:
        print("Try to re-connect camera, Please wait!")


