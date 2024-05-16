# -*- coding: utf-8 -*-
"""
Copyright (c) XYZ Robotics Inc. - All Rights Reserved
Unauthorized copying of this file, via any medium is strictly prohibited
Proprietary and confidential
Author: Yuhang Wu <yuhang.wu@xyzrobotics.ai>, 2022-04-14
"""
from datetime import datetime

import sqlalchemy.exc
from flask import Blueprint

from apps import openapi
from apps.base_app.flask_hook import req_log
from apps.exceptions import XYZIntegrityError, XYZNotFoundError
from apps.utils.validation import validator
from apps.models import db
from apps.programs.dpt.views.map_box.entity import MapBoxEntity
from apps.programs.dpt.views.map_box.models import MapBoxDBModel
from apps.programs.dpt.views.map_box.schemas import (
    SingleResponse,
    PaginationResponse,
    MapBoxCreateSchema,
    MapBoxDeleteSchema,
    MapBoxQuerySchema,
    MapBoxUpdateSchema
)

bp = Blueprint("map_box", __name__, url_prefix="/api/dpt/map_box")


@bp.route("/<serial_number>/", methods=["GET"])
@req_log()
@validator()
@openapi.api_doc(tags=["XLHB(标准库接口汇总)", "纸箱注册"])
def get_box(serial_number: str):
    """获取纸箱"""
    # 只查询一个纸箱编号
    map_box = (db.session.query(MapBoxDBModel).filter_by(
        serial_number=serial_number
    ).first())
    if not map_box:
        raise XYZNotFoundError(error_message="未知纸箱编号")
    return SingleResponse(data=MapBoxEntity.from_orm(map_box))


@bp.route("/", methods=["GET", "POST"])
@req_log()
@validator()
@openapi.api_doc(tags=["XLHB(标准库接口汇总)", "纸箱注册"], request_body=MapBoxQuerySchema)
def list_box(body: MapBoxQuerySchema):
    """搜索纸箱

    接口文档: https://xyz-robotics.atlassian.net/wiki/spaces/depalletizeproduct/pages/475562620/HMI#%E6%9F%A5%E8%AF%A2%E7%BA%B8%E7%AE%B1

    Args:
        body: 请求体
            MapBoxQuerySchema可组合多个过滤条件.

            基本结构:
                {
                  "page": 1,                    # 当前页码
                  "page_size": 20,              # 页面大小
                  "single": false,              # 是否仅返回单条数据
                  "filters": [                  # 过滤条件
                    {
                      "op": "gt",               # 操作符
                      "field": "real_length",   # 字段名
                      "value": 50               # 具体值
                    }
                  ],
                  "sort": [
                    {
                      "field": "real_length",   # 字段名
                      "collation": 0            # 排序规则
                    }
                  ]
                }

            例1:
                查询serial_number = "123"的记录
                {
                    "filters": [
                        {
                            "op": "eq",
                            "field": "serial_number",
                            "value": "123"
                        }
                    ]
                }

            例2:
                查询满足 1 < real_length < 5 的记录
                {
                    "filters": [
                        {
                            "op": "gt",
                            "field": "real_length",
                            "value": 1
                        },
                        {
                            "op": "lt",
                            "field": "real_length",
                            "value": 5
                        }
                    ]
                }

    Returns:
        返回查询结果

        output example:
        ---
        {
            "code": 0,
            "message": "success",
            "page": 1,
            "page_size": 20,
            "total_page": 4,
            "count": 80,
            "data": [
                {
                    "serial_number": "44363489",
                    "normal_length": 87,
                    "normal_width": 90,
                    "normal_height": 43,
                    "real_length": 74,
                    "real_width": 86,
                    "real_height": 62
                },
            ]
        }
        ---
    """
    filters = body.to_sa_filters()
    collation = body.to_sa_collation()
    count = db.session.query(MapBoxDBModel).filter(*filters).count()
    queryset = db.session.query(MapBoxDBModel) \
        .filter(*filters) \
        .order_by(*collation) \
        .limit(body.page_size) \
        .offset((body.page - 1) * body.page_size)
    if body.single:
        ret = queryset.first()
        if ret is None:
            # 未找到记录
            data = []
        else:
            data = queryset.first().entity
            return SingleResponse(data=data)
    else:
        data = [MapBoxEntity.from_orm(q) for q in queryset]
    return PaginationResponse(
        count=count,
        page_size=body.page_size,
        page=body.page,
        data=data
    )


@bp.route("/add", methods=["POST"])
@req_log()
@validator()
@openapi.api_doc(tags=["XLHB(标准库接口汇总)", "纸箱注册"], request_body=MapBoxCreateSchema)
def create_box(body: MapBoxCreateSchema):
    """创建一条记录

    Args:
        body: 用于创建纸箱的实体结构
    """
    map_box = MapBoxDBModel.from_entity(body)
    try:
        db.session.add(map_box)
        db.session.commit()
    except sqlalchemy.exc.IntegrityError as err:
        db.session.rollback()
        raise XYZIntegrityError(
            error_message="添加失败(纸箱已存在), 请检查是否存在重复的长宽高或纸箱编号"
        ) from err
    return SingleResponse(data=MapBoxEntity.from_orm(map_box))


@bp.route("/edit", methods=["POST"])
@req_log()
@validator()
@openapi.api_doc(tags=["XLHB(标准库接口汇总)", "纸箱注册"], request_body=MapBoxUpdateSchema)
def update_box(body: MapBoxUpdateSchema):
    """更新一条记录

    Args:
        body: 用于更新纸箱的实体结构
    """
    map_box = db.session.query(MapBoxDBModel).filter(
        MapBoxDBModel.serial_number == body.serial_number,
        MapBoxDBModel.is_del == False,
    ).first()
    if map_box is None:
        raise XYZNotFoundError(error_message="未知纸箱编号")

    # 全量更新
    for key, val in body.dict(exclude_unset=True).items():
        setattr(map_box, key, val)
    map_box.update_time = datetime.now()

    # 存储
    try:
        db.session.commit()
        return SingleResponse(data=MapBoxEntity.from_orm(map_box))
    except Exception as exc:
        db.session.rollback()
        raise exc


@bp.route("/delete", methods=["POST", "DELETE"])
@req_log()
@validator()
@openapi.api_doc(tags=["XLHB(标准库接口汇总)", "纸箱注册"], request_body=MapBoxDeleteSchema)
def delete(body: MapBoxDeleteSchema):
    """删除一条记录

    给定serial_number并删除

    Args:
        body(MapBoxDeleteSchema): 请求体, 包含必填项serial_number
    """
    map_box = db.session.query(MapBoxDBModel).filter(
        MapBoxDBModel.serial_number == body.serial_number,
        MapBoxDBModel.is_del == False,
    ).first()
    if not map_box:
        raise XYZNotFoundError(error_message="未知纸箱编号")
    map_box.is_del = None
    try:
        db.session.commit()
    except Exception as exc:
        db.session.rollback()
        raise exc
    return SingleResponse(data=MapBoxEntity.from_orm(map_box))
