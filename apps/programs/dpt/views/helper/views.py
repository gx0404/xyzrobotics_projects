#!/usr/bin/env python
# coding=utf-8
# from __future__ import unicode_literals
"""
Copyright (c) XYZ Robotics Inc. - All Rights Reserved.
Unauthorized copying of this file, via any medium is strictly prohibited.
Proprietary and confidential.
Author: Michael Yung Shan Su <michael.su@xyzrobotics.ai>, Dec. 2019.
"""

import datetime

from flask import Blueprint, request

from apps import openapi
from apps.base_app.flask_hook import req_log
from apps.enums import ErrorSources
from apps.ext.manager import TaskManager
from apps.globals import mp
from apps.helpers import make_json_response, record_error
from apps.models import db
from apps.programs.dpt.views.helper.models import AssistTask, ExpImage
from apps.programs.dpt.views.helper.website_utils import save_to_database
from apps.std_log_msg.general_log_msg.task_planner_msg import call_remote_helper

bp = Blueprint("helper", __name__, url_prefix="/api/helper")


@bp.route("/post", methods={"POST"})
@req_log(ignoreReq=True)
@openapi.api_doc(tags=["XLHB(标准库接口汇总)", "视觉标注助手"], summary="请求视觉标注")
def help_req():
    """Handle help request from client

    Form:
        ---
        {
            "timestamp_request": "",
            "msg": "",
            "label_method": "",
            "image_path": "",
            "camera_id": "",
            "annotation_json_string": "",
            "img": File()
        }
        ---

    Returns:
        ---
        {"task_id": "123"}
        ---
    """
    request_data = request.form
    task_id = save_to_database(request_data, request.files)
    if task_id is None:
        return {"request_status": "Image storage failed"}
    # to HMI
    mp.system(call_remote_helper)

    task_manager = TaskManager._get_subinstance()
    if task_manager is None:
        # 兼容 V1.3.0 及之前版本，通过导包的方式获取 task_manager
        try:
            from wcs_adaptor.manager import task_manager
        except ImportError as e:
            raise ImportError("TaskManager is not initialized.") from e
    task = task_manager.first()
    # 记录异常
    record_error(
        code=call_remote_helper["code"],
        msg="视觉异常",
        source=ErrorSources.VISION,
        task=task,
        tip=call_remote_helper["zh_tip"],
    )

    return {"task_id": task_id}


@bp.route("/result", methods={"POST"})
@req_log()
@openapi.api_doc(tags=["XLHB(标准库接口汇总)", "视觉标注助手"], summary="查询标注结果")
def result_req():
    """Handle result request from client

    Form:
        ---
        {"timeout": 10, "task_id": "123"}
        ---

    Returns:
        ---
        {"state": "expired", "error_code": "0", "error_msg": "", "annotation_json_string": ""}
        ---
    """
    request_data = request.form
    message = {}
    task = AssistTask.query.get(request_data["task_id"])
    message["state"] = task.state
    if task.state == "finish":
        task.timestamp_response = datetime.datetime.now()
        exp_images = ExpImage.query.filter_by(task=task)
        for exp_image in exp_images:
            if task.label_method == "box_label":
                message["annotation_json_string"] = exp_image.annotation_json_string

    else:
        timeout = None
        if "timeout" in request_data and request_data["timeout"]:
            timeout = int(request_data["timeout"])
        if (
            timeout is not None
            and datetime.datetime.now() - task.timestamp_request
            > datetime.timedelta(seconds=timeout)
        ):
            # time out
            task.state = "expired"
            task.error_code = -1
            task.error_msg = "Fail to request local help result"
            db.session.commit()
            message["state"] = "expired"
            # return {
            #     "state": "Unfinished",
            #     "request_status": "Unfinished in limited time"
            # }
    message["error_code"] = task.error_code
    message["error_msg"] = task.error_msg
    return message


# TO WEB
@bp.route("/request_label/", methods=["GET"])
@req_log()
@openapi.api_doc(tags=["XLHB(标准库接口汇总)", "视觉标注助手"], summary="开始标注最近的一个任务")
def request_to_label():
    """Get request from frontend, return task

    Returns:
        ---
        {
            "code": 0,
            "msg": "success",
            "data": {
                "task_id": "123",
                "label_method": "",
                "image_url": "",
                "camera_id": "",
                "msg": "",
                "annotation_json_string": ""
                }
        }
        ---
    """
    task = (
        AssistTask.query.filter(AssistTask.state.in_(["unassigned", "labeling"]))
        .order_by(AssistTask.task_id.desc())
        .first()
    )
    if task:
        if task.state == "unassigned":
            task.state = "labeling"
            task.timestamp_assign = datetime.datetime.now()
            db.session.commit()

        task_img = ExpImage.query.filter_by(task=task).first()
        message = {
            "task_id": task.task_id,
            "label_method": task.label_method,
            "image_url": task_img.image_url,
            "camera_id": task_img.camera_id,
            "msg": task.msg,
            "annotation_json_string": task_img.annotation_json_string,
        }
        return make_json_response(data=message)
    else:
        return make_json_response(code=-1, msg="No task available")


# TO WEB
@bp.route("/finish_label", methods=["GET", "POST"])
@req_log()
@openapi.api_doc(tags=["XLHB(标准库接口汇总)", "视觉标注助手"], summary="结束标注")
def finish_label():
    """

    Body:
        ---
        {
            "task_id": "123",
            "error_code": "",
            "error_msg": ""
        }
        ---
    """
    request_data = request.form

    if "task_id" not in request_data:
        request_data = request.json
        if "task_id" not in request_data:
            message = {"result": "No task to send", "result_zh": "传送失败！您并未传送任何结果"}
            return make_json_response(code=-1, data=message)

    task = AssistTask.query.get(request_data["task_id"])
    # Save to database

    if task.state == "finish" or task.state == "expired":
        message = {
            "result": "Fail. The task was finished or expired.",
            "result_zh": "任务失败！此任务已超过时间",
        }
        return make_json_response(code=-1, data=message)

    task.state = "finish"
    task.error_code = request_data["error_code"]
    task.error_msg = request_data["error_msg"]
    task.timestamp_finish = datetime.datetime.now()

    if task.label_method == "box_label":
        exp_img = ExpImage.query.filter_by(task=task).first()
        exp_img.annotation_json_string = request_data.get("annotation_json_string", "")

    db.session.commit()

    message = {
        "result": "Successfully sending result to server",
        "result_zh": "任务成功！您已成功完成标注并送出",
    }
    return make_json_response(data=message)


@bp.route("/cancel_assign", methods=["GET"])
@req_log()
@openapi.api_doc(tags=["XLHB(标准库接口汇总)", "视觉标注助手"], summary="取消标注")
def cancel_assign():
    """

    Returns:
        ---
        {"task_id": "123"}
        ---
    """
    request_data = request.values
    task_id = request_data["task_id"]
    task = AssistTask.query.filter_by(task_id=task_id)[0]
    if task.state == "labeling":
        task.state = "unassigned"
        task.timestamp_assign = datetime.datetime.now()
        task.assistant_name = None
        db.session.commit()
    return {}
