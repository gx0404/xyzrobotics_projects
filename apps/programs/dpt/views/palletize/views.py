# -*- coding: utf-8 -*-

"""
Copyright (c) XYZ Robotics Inc. - All Rights Reserved
Unauthorized copying of this file, via any medium is strictly prohibited
Proprietary and confidential
Author: Michael Su <michael.su@xyzrobotics.ai>, 5/6/2022
"""
import requests
from flask import Blueprint

from apps import openapi
from apps.log import outside_log
from apps.settings import settings

STATIC_FOLDER = settings.STATIC_FOLDER

bp = Blueprint("palletize", __name__, url_prefix="/api/dpt")


@bp.route("/add_box", methods=["GET", "POST"])
@openapi.api_doc(tags=["XLHB(标准库接口汇总)", "拆码垛(标准库内置)"])
def add_box():
    # """在workspace上增加当前纸箱,HMI码垛异常处理中，点击增加纸箱，会在当前环境中增加纸箱，同时发送完成1个

    # Returns:
    #     ---
    #     {
    #         "code": 0,
    #         "message": ""
    #     }
    #     ---
    # """
    # try:
    #     from xyz_env_manager.msg import (
    #         AttachedCollisionObject,
    #         BookedPickItems,
    #         BookedPlaceItems,
    #     )
    #     from xyz_env_manager.client import (
    #         get_all_booked_place_items,
    #         get_all_booked_pick_items,
    #     )
    #     from xyz_env_manager.client import (
    #         update_finished_planned_items,
    #         add_container_items,
    #     )

    #     update_finished_planned_items(
    #         get_all_booked_place_items()[0].workspace_id,
    #         get_all_booked_place_items()[0].planned_item_ids,
    #     )
    #     add_container_items(
    #         get_all_booked_place_items()[0].workspace_id,
    #         get_all_booked_place_items()[0].placed_items,
    #     )
    # except Exception:
    #     outside_log.warn("xyz_env_manager error")
    #     output_data = {"code": -1, "message": "xyz_env_manager error"}
    #     return output_data

    # url = "http://127.0.0.1:7002/api/rafcon/manual_add_box"
    # data = {"pick_num": 1}
    # try:
    #     requests.post(url, json=data).json()
    # except Exception:
    #     outside_log.warn("report_task_status error")
    #     output_data = {"code": -1, "message": "report_task_status error"}
    #     return output_data

    # 系统运行状况
    output_data = {"code": 0, "message": "ok"}
    return output_data


@bp.route("/remove_box", methods=["GET", "POST"])
@openapi.api_doc(tags=["XLHB(标准库接口汇总)", "拆码垛(标准库内置)"])
def remove_box():
    """在workspace上减去当前纸箱,HMI码垛异常处理中，点击减少纸箱，会在当前环境中减去纸箱

    Returns:
        ---
        {
            "code": 0,
            "message": ""
        }
        ---
    """
    # 系统运行状况
    output_data = {"code": 0, "message": "ok"}
    return output_data
