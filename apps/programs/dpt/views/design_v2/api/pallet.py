# -*- coding: utf-8 -*-
"""
Copyright (c) XYZ Robotics Inc. - All Rights Reserved
Unauthorized copying of this file, via any medium is strictly prohibited
Proprietary and confidential
Author: Yuhang Wu <yuhang.wu@xyzrobotics.ai>, 2022-07-26
"""
from flask import Blueprint

from apps import openapi
from apps.base_app.flask_hook import req_log
from apps.exceptions import XYZIntegrityError
from apps.models import start_transaction
from apps.utils.validation import validator
from ..crud import crud_pallet, crud_plan
from ..schemas import (
    GetPalletResponse,
    ListPalletResponse,
    PalletCreateSchema,
    PalletQuerySchema,
    PalletUpdateSchema,
)

bp = Blueprint("dpt.design.v2.pallet", __name__, url_prefix="/api/dpt/design/v2/pallet")


@bp.route("/search", methods=["POST"])
@req_log()
@validator()
@openapi.api_doc(
    tags=["XLHB(标准库接口汇总)", "垛型规划"],
    request_body=PalletQuerySchema,
    response_body=ListPalletResponse,
)
def list_pallets(body: PalletQuerySchema):
    """查询托盘."""
    data = crud_pallet.pagination(query=body)
    return ListPalletResponse(**data)


@bp.route("/<id>", methods=["PUT"])
@req_log()
@validator()
@openapi.api_doc(
    tags=["XLHB(标准库接口汇总)", "垛型规划"],
    request_body=PalletUpdateSchema,
    response_body=GetPalletResponse,
)
def edit_pallet(id: int, body: PalletUpdateSchema):
    """更新托盘."""
    with start_transaction() as session:
        pallet = crud_pallet.update(session=session, pk=id, update=body)
    return GetPalletResponse(data=pallet)


@bp.route("/", methods=["POST"])
@req_log()
@validator()
@openapi.api_doc(
    tags=["XLHB(标准库接口汇总)", "垛型规划"],
    request_body=PalletCreateSchema,
    response_body=GetPalletResponse,
)
def create_pallet(body: PalletCreateSchema):
    """新增托盘记录."""
    with start_transaction() as session:
        pallet = crud_pallet.create(session=session, create=body)
    return GetPalletResponse(data=pallet)


@bp.route("/<id>", methods=["DELETE"])
@req_log()
@validator()
@openapi.api_doc(
    tags=["XLHB(标准库接口汇总)", "垛型规划"],
    response_body=GetPalletResponse,
)
def delete_pallet(id: int):
    """删除托盘记录."""
    with start_transaction() as session:
        pallet = crud_pallet.get_or_404(session=session, pk=id)
        # 存在则不删除.
        if crud_plan.is_exists_by_pallet(session=session, pallet=pallet):
            raise XYZIntegrityError(error_message="删除失败, 该托盘仍存在绑定的规划记录.")
        else:
            pallet = crud_pallet.delete(session=session, pk=id)
    return GetPalletResponse(data=pallet)
