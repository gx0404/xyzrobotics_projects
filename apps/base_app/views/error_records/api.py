#  -*- coding: utf-8 -*-
#  Copyright (c) XYZ Robotics Inc. - All Rights Reserved
#  Unauthorized copying of this file, via any medium is strictly prohibited
#  Proprietary and confidential
#  Author: Yuhang Wu <yuhang.wu@xyzrobotics.ai>, 2022/8/30 上午10:28
from flask import Blueprint

from apps import openapi
from apps.utils.validation import validator
from apps.base_app.flask_hook import req_log
from apps.base_app.views.error_records.schemas import (
    ErrorRecordQuerySchema,
    SingleErrorRecordsResponse
)

from .crud import crud_error_records
from .schemas import MultiErrorRecordsResponse

bp = Blueprint("error_records", __name__, url_prefix="/api/error_records")


@bp.route("/search/", methods=["POST"])
@req_log()
@validator()
@openapi.api_doc(tags=["XLHB(标准库接口汇总)", "异常记录"], request_body=ErrorRecordQuerySchema)
def list_errors(body: ErrorRecordQuerySchema):
    """查询异常

    Returns:
        ---
        {"code": 0, "msg": "success", "data": []}
        ---
    """
    data = crud_error_records.pagination(query=body)
    return MultiErrorRecordsResponse(**data)


@bp.route("/<id>/")
@req_log()
@validator()
@openapi.api_doc(tags=["XLHB(标准库接口汇总)", "异常记录"])
def get_error(id: int):
    """查看异常详情
    
    Returns:
        ---
        {
            "code": 0,
            "msg": "success",
            "data": {
                "id": 1,
                "task_id": "123",
                "task": {},
                "error_code": "xx",
                "error_msg": "xx",
                "error_source": "xtf",
                "tip": "y",
                "create_time": "2022-01-01 00:00:00"
            }
        }
        ---
    """
    record = crud_error_records.get_or_404(pk=id)
    return SingleErrorRecordsResponse(data=record)
