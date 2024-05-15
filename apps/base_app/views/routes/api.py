# -*- coding: utf-8 -*-
"""
Copyright (c) XYZ Robotics Inc. - All Rights Reserved
Unauthorized copying of this file, via any medium is strictly prohibited
Proprietary and confidential
Author: Yuhang Wu <yuhang.wu@xyzrobotics.ai>, 2022-07-15

路由规则，用于前端动态渲染.
"""
from flask import Blueprint

from apps import openapi
from apps.base_app.flask_hook import req_log as catch_log
from apps.utils.validation import validator
from .crud import crud_route
from .schema import ListResponse, Response, RouteQuerySchema, RouteUpdateSchema

bp = Blueprint("routes", __name__, url_prefix="/api/routes")


# TODO: openapi 当前不支持 parameters 参数
@bp.route("/", methods=["GET"])
@catch_log()
@validator()
@openapi.api_doc(tags=["XLHB(标准库接口汇总)", "路由"])
def list_routes(query: RouteQuerySchema):
    """查询当前项目类型可展示的路由

    Returns:
        ---
        {
            "code": 0,
            "msg": "success",
            "data": [
                {
                    "route_id": "",
                    "to": "path",
                }
            ]
        }
        ---
    """
    routes = crud_route.filter_by_project_type(query)
    return ListResponse(data=routes)


@bp.route("/edit/", methods=["POST"])
@catch_log()
@validator()
@openapi.api_doc(tags=["XLHB(标准库接口汇总)", "路由"], request_body=RouteUpdateSchema)
def edit_routes(body: RouteUpdateSchema):
    """更新路由

    Args:
        body: {
            "route_id": "",
            "name": ""
        }

    Returns:
        ---
        {
            "code": 0,
            "msg": "success",
            "data": {
                "route_id": "",
                "to": "",
                "name": "",
                "project_type": "",
            }
        }
        ---
    """
    route = crud_route.update(update=body)
    return Response(data=route)
