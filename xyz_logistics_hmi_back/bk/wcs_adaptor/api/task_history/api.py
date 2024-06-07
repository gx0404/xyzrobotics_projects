# -*- coding: utf-8 -*-
"""
Copyright (c) XYZ Robotics Inc. - All Rights Reserved
Unauthorized copying of this file, via any medium is strictly prohibited
Proprietary and confidential
Author: Yuhang Wu <yuhang.wu@xyzrobotics.ai>, 2022-07-04

历史任务相关的接口.
"""
import io
import time

import pandas as pd
from flask import Blueprint, send_file

from apps import cached_app, catch_log, validator, wcs_log, openapi
from wcs_adaptor.api.task_history.crud import crud_task_history
from wcs_adaptor.api.task_history.schema import (
    ListResponse,
    Response,
    TaskHistoryQuerySchema
)

bp = Blueprint("task_history", __name__, url_prefix="/api/task_history")


@bp.route("/", methods=["POST"])
@catch_log(log=wcs_log)
@validator()
@openapi.api_doc(tags=["WCS"], request_body=TaskHistoryQuerySchema, summary="查询历史任务")
def list_tasks(body: TaskHistoryQuerySchema):
    """根据搜索条件返回多条记录."""
    data = crud_task_history.search(qs=body)

    if body.single and len(data) == 1:
        data = data[0]
        return Response(data=data)

    filters = body.to_sa_filters()
    count = crud_task_history.count(filters)

    return ListResponse(
        count=count,
        page_size=body.page_size,
        page=body.page,
        data=data
    )


@bp.route("/<id>/", methods=["GET"])
@catch_log(log=wcs_log)
@validator()
@openapi.api_doc(tags=["WCS"], summary="查询历史任务详情")
def get_task(id: int):
    """根据主键ID获取任务."""
    data = crud_task_history.get_task_or_404(id)
    return Response(data=data)


@bp.route("/download/", methods=["GET", "POST"])
@catch_log(log=wcs_log, ignoreRes=True)
@validator()
@openapi.api_doc(tags=["WCS"], summary="导出历史任务")
def download_task():
    """下载历史任务文件."""
    tasks = crud_task_history.all()
    records = [task.dict() for task in tasks]
    df = pd.DataFrame.from_records(records)
    text = df.to_csv(index=False)
    fp = io.BytesIO(text.encode(encoding="utf-8"))
    resp = send_file(
        filename_or_fp=fp,
        mimetype="text/csv",
        as_attachment=True,
        attachment_filename=f"history_task_{int(time.time())}.csv"
    )
    resp.headers["Access-Control-Allow-Headers"] = "*"
    resp.headers["Access-Control-Expose-Headers"] = "Content-Disposition"
    return resp


app = cached_app()
app.register_blueprint(bp)
