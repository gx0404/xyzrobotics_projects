# -*- coding: utf-8 -*-
"""
Copyright (c) XYZ Robotics Inc. - All Rights Reserved
Unauthorized copying of this file, via any medium is strictly prohibited
Proprietary and confidential

Author: Kun Chen <kun.chen@xyzrobotics.ai>, 2022-10-25
"""
import os
from flask import Blueprint, request

from apps import make_json_response, settings
from apps.base_app.flask_hook import req_log
from apps.log import wcs_log
from apps.utils.util import set_latest_vision_image_link
from apps.utils.validation import validator

bp = Blueprint("lowcode", __name__, url_prefix="/api/lowcode")


@bp.route("/latest_pictures", methods=["POST"])
@req_log(log=wcs_log)
@validator()
def get_latest_pictures():
    """获取今天相机拍照的最新的视觉图片
    
    Inputs:
        host: str, 对于客户端本服务的主机地址
        port: Union[str, int], (可选)返回的端口号地址，默认是本服务端口号
        date_format: str, (可选)视觉日志文件命名的日期格式
    
    """
    req_data = request.get_json()
    date_format = req_data.get("date_format", "%Y%m%d")

    data = { "image_list": [] }
    # 设置当前最新视觉图片的路径
    link_pathname = "temp_vision_images"
    if (src := set_latest_vision_image_link(
        static_folder=settings.STATIC_FOLDER,
        link_pathname=link_pathname,
        base_log_path="/home/xyz/xyz_log",
        date_format=date_format
    )) is None:
        return make_json_response(data=data)
    # 根据link_pathname，获取最新视觉图片地址
    host = req_data.get("host")
    port = req_data.get("port", settings.XYZ_HTTP_PORT)
    base_url = f"http://{host}:{port}/static/{link_pathname}"
    for item in sorted(os.listdir(src), reverse=True):
        if item.endswith(('jpg', 'png', )):
            data["image_list"].append(f"{base_url}/{item}")
            # 允许展示的最大图片数量，数量过大会影响前端加载
            if len(data["image_list"]) == 10:
                break

    return make_json_response(data=data)


@bp.route("/robot_status", methods=["GET"])
@req_log(log=wcs_log)
@validator()
def get_robot_status():
    """获取机器人运行状态

    Returns:
        data = {
            "robot_status": "停止"/"运行"/"暂停"/"故障"
        }
    """
    # from xyz_io_client.io_client import set_digit_output, get_digit_input
    # robot_status = get_digit_input("2", 30)
    # 获取到的机器人运行状态标志
    robot_status = "故障"

    data = {
        "robot_status": robot_status
    }

    return make_json_response(data=data)


@bp.route("/control_mode", methods=["GET"])
@req_log(log=wcs_log)
@validator()
def get_control_mode():
    """获取控制模式

    Returns:
        data = {
            "control_mode": "袋型"/"就地"/"远程"
        }
    """
    # 获取控制模式标志
    control_mode = "远程"

    data = {
        "control_mode": control_mode
    }

    return make_json_response(data=data)


@bp.route("/blanking_bags", methods=["GET"])
@req_log(log=wcs_log)
@validator()
def get_blanking_bags():
    """获取已下料袋数

    Returns:
        data = {
            "blanking_bags": 22
        }
    """
    # 获取已下料袋数
    blanking_bags = 333

    data = {
        "blanking_bags": blanking_bags
    }

    return make_json_response(data=data)


@bp.route("/set_bags", methods=["POST"])
@req_log(log=wcs_log)
@validator()
def set_blanking_bags():
    """下料袋数设置

    Args:
    bags(int): 下料袋数

    Returns:
        data = {
            "bags": 10
        }
    """
    # 通过参数bags获取前端设置的下料袋数
    req_data = request.get_json()
    bags = req_data.get("bags")

    data = {
        "bags": bags
    }

    return make_json_response(data=data)


@bp.route("/single_class_pal_task", methods=["POST"])
@req_log(log=wcs_log)
@validator()
def set_single_class_pal_task():
    """下发单码任务示例

    Args:
    task_id(str): 任务编号
    target_num(str): 任务数量
    from(str): 抓取工作空间
    to(str): 放置工作空间
    sku_id(str): 物料编号
    length(str): 长（毫米）
    width(str): 宽（毫米）
    height(str): 高（毫米）
    weight(str): 重（千克）

    Returns:
        data = {
            "task_id": "#test#001",
            "target_num": "15",
            "from": "00",
            "to": "11",
            "sku_info":{
                "sku_id": "10086",
                "length": "365",
                "width": "256",
                "height": "161",
                "weight": "0.6"
            }
        }
    """
    req_data = request.get_json()
    wcs_log.info(f"【示例接口 single_class_pal_task】接收到单码任务数据: {req_data}")
    # TODO: 这里可以实现处理接收请求的相关逻辑

    return make_json_response(data=req_data)
