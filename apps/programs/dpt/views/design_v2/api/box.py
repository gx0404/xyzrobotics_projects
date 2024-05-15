# -*- coding: utf-8 -*-
"""
Copyright (c) XYZ Robotics Inc. - All Rights Reserved
Unauthorized copying of this file, via any medium is strictly prohibited
Proprietary and confidential
Author: Yuhang Wu <yuhang.wu@xyzrobotics.ai>, 2022-07-26
"""
import os.path
from typing import List

import pydantic
from flask import Blueprint, request
from werkzeug.datastructures import ImmutableMultiDict

from apps import make_json_response, openapi
from apps.base_app.flask_hook import req_log
from apps.enums import MediaType
from apps.exceptions import XYZIntegrityError, XYZValidationError
from apps.models import start_transaction
from apps.responses import FileResponse
from apps.search_filter import QueryFilter
from apps.settings import APP_PATH
from apps.utils.upload_manager import check_upload_image
from apps.utils.validation import validator
from ..crud import crud_box, crud_plan
from ..entity import BoxEntity
from ..schemas import (
    BoxCreateSchema,
    BoxQuerySchema,
    BoxDeleteSchema,
    MultiBoxResponse,
    SingleBoxResponse,
    PaginationBoxResponse
)
from ..utils import check_duplicate_box, parse_box_file

bp = Blueprint(
    "dpt.design.v2.box",
    __name__,
    url_prefix="/api/dpt/design/v2/box"
)


@bp.route("/search", methods=["POST"])
@req_log()
@validator()
@openapi.api_doc(tags=["XLHB(标准库接口汇总)", "垛型规划"], request_body=BoxQuerySchema)
def list_boxes(body: BoxQuerySchema):
    """查询纸箱
    
    Returns:
        ---
        {
          "code": 0,
          "msg": "success",
          "page": 1,
          "page_size": 69,
          "total_page": 1,
          "count": 3,
          "data": [
            {
              "id": 11,
              "name": "1",
              "length": 1,
              "width": 1,
              "height": 1,
              "weight": 1,
              "scan_code": "1",
              "img_url": None
            }
          ]
        }
        ---
    """
    data = crud_box.pagination(query=body)
    return PaginationBoxResponse(**data)


@bp.route("/", methods=["POST"])
@req_log()
@validator()
@openapi.api_doc(tags=["XLHB(标准库接口汇总)", "垛型规划"])
def create_box():
    """创建纸箱"""
    # WARN: openapi 暂不支持 Form
    files: ImmutableMultiDict = request.files
    form = request.form
    try:
        box_create = BoxCreateSchema(
            name=form.get("name"),
            length=form.get("length"),
            width=form.get("width"),
            height=form.get("height"),
            weight=form.get("weight"),
            scan_code=form.get("scan_code"),
            img_file=files.get("img_file"),
        )
    except pydantic.ValidationError as e:
        raise XYZValidationError(error_message=str(e)) from e

    if box_create.img_file and not check_upload_image(box_create.img_file):
        return make_json_response(
            code=-1, msg="Image file type unsupported, support type:[jpg, png]"
        )

    with start_transaction() as session:
        box = crud_box.create(session, box_create)
    return SingleBoxResponse(data=box)


@bp.route("/<id>", methods=["DELETE"])
@req_log()
@validator()
@openapi.api_doc(tags=["XLHB(标准库接口汇总)", "垛型规划"])
def delete_box(id: int):
    """删除一个纸箱.

    如果被纸箱正在使用中，则删除失败
    """
    with start_transaction() as session:
        box = crud_box.get_or_404(session=session, pk=id)
        # 存在则不删除.
        if crud_plan.is_exists_by_box(session=session, box=box):
            raise XYZIntegrityError(error_message="删除失败, 该纸箱仍存在绑定的规划记录.")
        else:
            box = crud_box.delete(session=session, pk=id)
    return SingleBoxResponse(data=box)


@bp.route("/", methods=["DELETE"])
@req_log()
@validator()
@openapi.api_doc(tags=["XLHB(标准库接口汇总)", "垛型规划"], request_body=BoxDeleteSchema)
def delete_boxes(body: BoxDeleteSchema):
    """删除多个纸箱记录.

    仍有规划绑定的纸箱记录将会删除失败，最终返回删除成功的纸箱记录.

    .. versionadded:: v1.4.0
        删除多个纸箱记录.
    """
    delete_list = []
    with start_transaction() as session:
        query = BoxQuerySchema(
            page_size=len(body.ids),
            filters=[QueryFilter(op="in", field="id", value=body.ids)],
        )
        boxes: List[BoxEntity] = crud_box.search(session=session, query=query)
        for box in boxes:
            # 存在则不删除.
            if not crud_plan.is_exists_by_box(session=session, box=box):
                crud_box.delete(session=session, pk=box.id)
                delete_list.append(box)
    return MultiBoxResponse(data=delete_list)


@bp.route("/clear_all", methods=["DELETE"])
@req_log()
@validator()
@openapi.api_doc(tags=["XLHB(标准库接口汇总)", "垛型规划"])
def clear_all():
    """删除所有纸箱记录.
    
    Returns:
        ---
        {
            "code": 0,
            "msg": "message",
            "data": {
                "deleted_count": 10
            }
        }
        ---
    """
    with start_transaction() as session:
        # 清空未被使用的纸箱
        deleted_count = crud_box.delete_all_unused(session)
        return make_json_response(data={"deleted_count": deleted_count})


@bp.route("/parse", methods=["POST"])
@req_log()
@validator()
@openapi.api_doc(tags=["XLHB(标准库接口汇总)", "垛型规划"])
def parse_file():
    """解析纸箱表格文件."""
    box_file = request.files.get("box_file")
    boxes = parse_box_file(box_file)
    # result = check_duplicate_box(boxes)
    # if result:
    #     raise XYZValidationError(
    #         error_message=f"当前批次存在重复数据({result.length} * {result.width} * {result.height})."
    #     )
    return SingleBoxResponse(data=boxes)


@bp.route("/download_template/", methods=["GET"])
@req_log(ignoreRes=True)
@validator()
@openapi.api_doc(tags=["XLHB(标准库接口汇总)", "垛型规划"])
def download_template():
    """下载纸箱模板文件."""
    return FileResponse(
        path=os.path.join(APP_PATH, "statics/box_template.xlsx"),
        media_type=MediaType.xlsx,
        as_attachment=True,
    )
