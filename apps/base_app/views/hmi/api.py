# -*- coding:utf-8 -*-
'''
Copyright (c) XYZ Robotics Inc. - All Rights Reserved
Unauthorized copying of this file, via any medium is strictly prohibited
Proprietary and confidential
Author: Michael Su <michael.su@xyzrobotics.ai>, June 2, 2022
'''
import os
import json
import base64

import rospy
from flask import Blueprint, request

from apps import openapi
from apps.settings import settings
from apps.base_app.flask_hook import req_log
from apps.exceptions import XYZBaseError
from apps.helpers import make_json_response
from apps.utils import plc

hmi_bp = Blueprint("hmi", __name__, url_prefix="/api/hmi")

qrcode = None


@hmi_bp.route("/config", methods=["GET"])
@openapi.api_doc(tags=["XLHB(标准库接口汇总)", "HMI"])
def get_hmi_config():
    """HMI获取其需要的一些前端配置，比如项目类型，项目名称，锁屏密码等"""
    data = {
        "project_type": settings.PROJECT_TYPE,
        "project_name": settings.PROJECT_NAME,
        "screen_lock_password": settings.SCREEN_LOCK_PASSWORD,
    }
    return make_json_response(data=data)


@hmi_bp.route("/feedback_qrcode", methods=["GET"])
@openapi.api_doc(tags=["XLHB(标准库接口汇总)", "HMI"])
def get_feedback_qrcode():
    """返回反馈二维码
    """
    global qrcode
    if qrcode is None:
        path = os.path.dirname(os.path.abspath(__file__))
        with open(f'{path}/FeedbackQrcode.png', 'rb') as image_file:
            qrcode = base64.b64encode(image_file.read()).decode()
    return make_json_response(data=qrcode)


@hmi_bp.route("/customized_hmi", methods=["POST"])
@req_log()
@openapi.api_doc(tags=["XLHB(标准库接口汇总)", "HMI"])
def get_customized_hmi():
    """获取辅助功能的配置

    Returns:
        ---
        {
            "code": 0,
            "msg": "success",
            "data": { 
                "ui": True,
                "url_prefix": "/api/custom/",
                "content": [
                {
                    "func_type": 1,
                    "route": "delete_task",
                    "button": {
                        "label": "delete task"
                    },
                    "helper": "XXXXXXXXX",  #Helper could be None
                },
                {
                    "func_type": 2,
                    "route": "manual_place_box",
                    "button": { "label": "check" },
                    "input": {
                        "label": "manual place box",
                        "key": "num",
                        "input_type": "int",
                        "default_value": None,
                    },
                    "helper": "XXXXX", 
                },    
                ]
            }
        }
        ---
    """
    import pymysql
    with open(settings.CUSTOM_HMI_CG, "r") as fin:
        custom_hmi_params = json.load(fin)
    
    #获取更新中欧笼车层数    
    content = custom_hmi_params["content"]    
    for custom_config in content:
        if custom_config["id"] == "set_cage_layer":
            set_cage_layer_config = custom_config
            break

    # 连接到 MySQL 数据库
    conn = pymysql.connect(
        host='localhost',
        user='root',
        password='xyz123456',
        database='hmi'
    )
    # 创建游标对象
    cursor = conn.cursor()

    # 执行 SQL 查询语句
    query = "select * from cage_layer"
    cursor.execute(query)
    
    # 获取查询结果
    cage_layer_results = cursor.fetchall()    
    set_cage_layer_config["default_value"] = cage_layer_results[0][1]
        
    return make_json_response(data=custom_hmi_params)


@hmi_bp.route("/close_buzzer", methods=["POST"])
@req_log()
@openapi.api_doc(tags=["XLHB(标准库接口汇总)", "HMI"])
def close_buzzer():
    """关闭蜂鸣器

    Returns:
        ---
        {"code": 0, "msg": "蜂鸣器已关闭"}
        ---
    """
    try:
        plc.close_buzzer()
        # from wcs_adaptor.depalletize.rafcon import init_error
        # init_error()
    except rospy.service.ServiceException as exc:
        raise XYZBaseError(error_message="蜂鸣器关闭失败, io_node 服务不可用") from exc
    except Exception as exc:
        raise XYZBaseError(error_message="蜂鸣器关闭失败") from exc
    else:
        return make_json_response(msg="蜂鸣器已关闭")


@hmi_bp.route("/update_lowcode", methods=["POST"])
@req_log()
@openapi.api_doc(tags=["XLHB(标准库接口汇总)", "HMI"])
def update_lowcode_config():
    """备份自定义页面JSON配置

    Returns:
        ---
        {"code": 0, "msg": "备份配置成功"}
        ---
    """
    req_data = request.get_json()
    lowcode_config = req_data.get("lowcode")
    config_path = settings.LOWCODE_CONFIG_PATH
    try:
        with open(config_path, "w", encoding="utf-8") as f:
            f.write(json.dumps(
                        json.loads(lowcode_config), 
                        indent=2, 
                        ensure_ascii=False))
    except Exception as exc:
        raise XYZBaseError(error_message="备份配置写入失败") from exc
    else:
        return make_json_response(msg="备份配置成功")
