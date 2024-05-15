# -*- coding: utf-8 -*-
"""
Copyright (c) XYZ Robotics Inc. - All Rights Reserved
Unauthorized copying of this file, via any medium is strictly prohibited
Proprietary and confidential
Author: Yuhang Wu <yuhang.wu@xyzrobotics.ai>, 2022-07-26
"""
import contextlib
import uuid

import pydantic
from flask import Blueprint, request
from werkzeug.datastructures import ImmutableMultiDict

from apps import hmi_log, make_json_response, openapi
from apps.base_app.flask_hook import req_log
from apps.enums import MediaType
from apps.exceptions import XYZBaseError, XYZValidationError
from apps.models import start_transaction
from apps.programs.dpt.views.design_v2.enums import BarcodeDirection
from apps.responses import StreamingResponse
from apps.utils.dt import now_timestamp
from apps.utils.upload_manager.upload_manager import remove_image, save_image
from apps.utils.validation import validator

from ..crud import crud_box, crud_pallet, crud_plan
from ..entity import Objects, PlanEntity
from ..schemas import (
    BatchGeneratePlanInputSchema,
    GeneratePlanInputSchema,
    MultiGenerateResultsResponse,
    OneGenerateResultResponse,
    PlanCreateSchema,
    PlanDeleteSchema,
    PlanDownloadSchema,
    PlanPaginationResponse,
    PlanPatchSchema,
    PlanQuerySchema,
    PlanSummaryOutputSchema,
    PlanUpdateSchema,
    SinglePlanResponse,
)
from ..schemas.plan import MultiPlanResponse
from ..utils import (
    check_duplicate_box,
    generate_plan_excel_to_bytes,
    generate_suggest_layout,
    parse_box_file,
    validate_plan,
)

bp = Blueprint("dpt.design.v2.plan", __name__, url_prefix="/api/dpt/design/v2/plan")


@bp.route("/search/", methods=["POST"])
@req_log()
@validator()
@openapi.api_doc(
    tags=["XLHB(标准库接口汇总)", "垛型规划"],
    request_body=PlanQuerySchema,
    response_body=PlanPaginationResponse,
)
def list_plans(body: PlanQuerySchema):
    """列出符合查询条件的规划."""
    data = crud_plan.pagination(query=body)
    for i, _data in enumerate(data["data"]):
        data["data"][i] = PlanSummaryOutputSchema.from_entity(_data)
    return PlanPaginationResponse(**data)


@bp.route("/no_image/", methods=["GET"])
@req_log()
@validator()
@openapi.api_doc(
    tags=["XLHB(标准库接口汇总)", "垛型规划"],
)
def list_no_image_plans():
    """获取所有没有图片的规划记录."""
    entities = crud_plan.get_no_image_plans()
    return MultiPlanResponse(data=entities)


@bp.route("/<id>")
@req_log()
@validator()
@openapi.api_doc(
    tags=["XLHB(标准库接口汇总)", "垛型规划"],
    response_body=SinglePlanResponse,
)
def get_plan(id: int):
    """根据ID获取一个规划结果."""
    with start_transaction() as session:
        plan = crud_plan.get_or_404(session=session, pk=id)
        plan.last_id = crud_plan.get_last(session=session, pk=id)
        plan.next_id = crud_plan.get_next(session=session, pk=id)
    return SinglePlanResponse(data=plan)


@bp.route("/", methods=["POST"])
@req_log()
@validator()
@openapi.api_doc(
    tags=["XLHB(标准库接口汇总)", "垛型规划"],
    response_body=SinglePlanResponse,
)
def add_plan():
    """添加跺型."""
    form = request.form
    image_file = request.files.get("image")
    image_url = None

    if image_file:
        filename = ".".join(
            [uuid.uuid4().__str__(), image_file.filename.rsplit(".", 1)[-1]]
        )
        image_url = save_image(image_file, filename=filename, dir_name="pallet_design")

    try:
        body = PlanCreateSchema(
            pallet_id=form.get("pallet_id"),
            box_id=form.get("box_id"),
            guillotine_packing=form.get("guillotine_packing"),
            barcode_direction=form.get("barcode_direction"),
            mirror=form.get("mirror"),
            flip=form.get("flip"),
            layers=form.get("layers"),
            layout=form.get("layout"),
            objects=form.get("objects"),
            image_url=image_url,
        )
    except pydantic.ValidationError as err:
        raise XYZValidationError(error_message=str(err)) from err

    with start_transaction() as session:
        box = crud_box.get_or_404(session=session, pk=body.box_id)
        pallet = crud_pallet.get_or_404(session=session, pk=body.pallet_id)
        if crud_plan.is_exists(session=session, box_id=box.id):
            raise XYZBaseError(error_message="当前纸箱垛型已存在")
        validate_plan(box, pallet, body.layers, body.objects.to_layout())
        plan = crud_plan.create(session=session, create=body)
        return SinglePlanResponse(data=plan)


@bp.route("/<id>", methods=["PUT"])
@req_log()
@validator()
@openapi.api_doc(
    tags=["XLHB(标准库接口汇总)", "垛型规划"],
    request_body=PlanUpdateSchema,
    response_body=SinglePlanResponse,
)
def edit_plan(id: int):
    """更新垛型."""
    form = request.form
    image_file = request.files.get("image")
    image_url = None
    old_image_url = None

    if image_file:
        plan = crud_plan.get_or_404(pk=id)
        old_image_url = plan.image_url
        filename = ".".join(
            [uuid.uuid4().__str__(), image_file.filename.rsplit(".", 1)[-1]]
        )
        image_url = save_image(image_file, filename=filename, dir_name="pallet_design")

    body = PlanUpdateSchema(
        pallet_id=form.get("pallet_id"),
        box_id=form.get("box_id"),
        # layout=form.get("layout"),
        objects=form.get("objects"),
        mirror=form.get("mirror"),
        image_url=image_url,
    )

    with start_transaction() as session:
        box = crud_box.get_or_404(session=session, pk=body.box_id)
        pallet = crud_pallet.get_or_404(session=session, pk=body.pallet_id)
        validate_plan(box, pallet, body.layers, body.objects.to_layout())
        plan = crud_plan.update(session=session, update=body, pk=id)
        # 删除历史文件.
        if old_image_url:
            with contextlib.suppress(FileNotFoundError):
                remove_image(old_image_url)
        return SinglePlanResponse(data=plan)


@bp.route("/<id>", methods=["PATCH"])
@req_log()
@validator()
@openapi.api_doc(
    tags=["XLHB(标准库接口汇总)", "垛型规划"],
    request_body=PlanUpdateSchema,
    response_body=SinglePlanResponse,
)
def patch_plan(id: int):
    """部分更新垛型.

    对于一条记录的部分数据进行更新，当前仅支持更改图片.
    """
    image_file = request.files.get("image")
    image_url = None
    old_image_url = None

    if image_file:
        plan = crud_plan.get_or_404(pk=id)
        # 判断历史记录是否有图片.
        if plan.image_url:
            # 取历史文件名.
            old_image_url = plan.image_url
        filename = ".".join(
            [uuid.uuid4().__str__(), image_file.filename.rsplit(".", 1)[-1]]
        )
        image_url = save_image(image_file, filename=filename, dir_name="pallet_design")

    patch_update = PlanPatchSchema(image_url=image_url)
    with start_transaction() as session:
        plan = crud_plan.patch_update(session=session, patch_update=patch_update, pk=id)
        # 删除历史文件.
        if old_image_url:
            remove_image(old_image_url)
        return SinglePlanResponse(data=plan)


@bp.route("/download/", methods=["POST", "GET"])
@req_log()
@validator()
@openapi.api_doc(
    tags=["XLHB(标准库接口汇总)", "垛型规划"],
    request_body=PlanDownloadSchema,
)
def download_plan(body: PlanDownloadSchema):
    """下载规划记录.

    单次最大导出 10000 条记录.
    ids 仅用于规划表的查询.
    filters 则用于纸箱表查询（作为子查询）.
    优先采用 ids 作为检索条件，如果 ids 被指定，则忽略 filters.
    """
    body.page_size = 10000
    if body.ids:
        entities = crud_plan.search(query=body)
    else:
        entities = crud_plan.search_by_box(query=body)
    hmi_log.info(f"Export Record Count: {len(entities)}")
    # 生成excel文件的字节流对象.
    content = generate_plan_excel_to_bytes(entities=entities)
    return StreamingResponse(
        filename=f"design-{now_timestamp()}.xlsx",
        stream=content,
        headers={"X-Record-Count": len(entities)},
        media_type=MediaType.xlsx,
        as_attachment=True,
    )


@bp.route("/<id>", methods=["DELETE"])
@req_log()
@validator()
@openapi.api_doc(
    tags=["XLHB(标准库接口汇总)", "垛型规划"],
    response_body=SinglePlanResponse,
)
def delete_plan(id: int):
    """删除一条记录."""
    with start_transaction() as session:
        delete_result = crud_plan.delete_or_404(session, pk=id)
    return SinglePlanResponse(data=delete_result)


@bp.route("/", methods=["DELETE"])
@req_log()
@validator()
@openapi.api_doc(tags=["XLHB(标准库接口汇总)", "垛型规划"], request_body=PlanDeleteSchema)
def delete_plans(body: PlanDeleteSchema):
    """多条规划记录."""
    with start_transaction() as session:
        delete_result = [crud_plan.delete(session=session, pk=pk) for pk in body.ids]
        # [None, None] -> []
        delete_result = list(filter(lambda x: x is not None, delete_result))
        return MultiPlanResponse(data=delete_result)


@bp.route("/clear_all", methods=["DELETE"])
@req_log()
@validator()
@openapi.api_doc(
    tags=["XLHB(标准库接口汇总)", "垛型规划"],
)
def clear_all():
    """删除所有垛型规划记录."""
    with start_transaction() as session:
        # 逻辑删除所有可见的垛型记录.
        deleted_count = crud_plan.delete_all(session=session, query=PlanQuerySchema())
        return make_json_response(data={"deleted_count": deleted_count})


@bp.route("/generate/", methods=["POST"])
@req_log()
@validator()
@openapi.api_doc(
    tags=["XLHB(标准库接口汇总)", "垛型规划"],
    request_body=GeneratePlanInputSchema,
)
def generate_plan(body: GeneratePlanInputSchema):
    """生成跺型."""
    box = crud_box.get_or_404(pk=body.box_id)
    pallet = crud_pallet.get_or_404(pk=body.pallet_id)
    if crud_plan.is_exists(box_id=box.id):
        raise XYZBaseError(error_message="当前纸箱垛型已存在")

    if (
        body.guillotine_packing == 1
        and body.barcode_direction == BarcodeDirection.DEFAULT
    ):
        raise XYZBaseError(error_message="回型垛的条码方向必须指定")

    layout = generate_suggest_layout(
        box=box,
        pallet=pallet,
        barcode_direction=body.barcode_direction,
        mirror=body.mirror,
        guillotine_packing=body.guillotine_packing,
        flip=body.flip,
    )
    objects = Objects.from_layout(layout)
    return OneGenerateResultResponse(
        data=PlanEntity(**body.dict(), id=0, objects=objects, box=box, pallet=pallet)
    )


@bp.route("/generates", methods=["POST"])
@req_log(ignoreRes=True)
@validator()
@openapi.api_doc(
    tags=["XLHB(标准库接口汇总)", "垛型规划"],
)
def generate_plans():
    """批量生成跺型.

    NOTICE: 批量生成垛型，会自动入库.
    """
    files: ImmutableMultiDict = request.files
    form: ImmutableMultiDict = request.form
    boxes = parse_box_file(files.get("box_file"))
    # if result := check_duplicate_box(boxes):
    #     raise XYZValidationError(
    #         error_message=f"当前批次存在重复数据({result.length} * {result.width} * {result.height})."
    #     )

    try:
        body = BatchGeneratePlanInputSchema(
            pallet_id=form.get("pallet_id"),
            guillotine_packing=form.get("guillotine_packing"),
            barcode_direction=form.get("barcode_direction"),
            mirror=form.get("mirror"),
            flip=form.get("flip"),
            layers=form.get("layers"),
            boxes=boxes,
        )
    except pydantic.ValidationError as err:
        raise XYZValidationError(error_message=str(err)) from err

    if not body.boxes:
        raise XYZBaseError(error_message="纸箱列表不能为空.")

    with start_transaction() as session:
        body_dict = body.dict()
        # layouts = []
        plan_create_list = []
        pallet = crud_pallet.get_or_404(session=session, pk=body.pallet_id)

        # 批量生成垛型.
        for box in body.boxes:
            box = crud_box.create(session=session, create=box)
            validate_plan(box, pallet, body.layers)
            layout = generate_suggest_layout(
                box=box,
                pallet=pallet,
                barcode_direction=body.barcode_direction,
                mirror=body.mirror,
                guillotine_packing=body.guillotine_packing,
                flip=body.flip,
            )
            objects = Objects.from_layout(layout)
            plan_create_list.append(
                PlanCreateSchema(
                    box_id=box.id,
                    objects=objects,
                    layers=pallet.max_height // box.height,
                    pallet_id=form.get("pallet_id"),
                    guillotine_packing=form.get("guillotine_packing"),
                    barcode_direction=form.get("barcode_direction"),
                    mirror=form.get("mirror"),
                    flip=form.get("flip"),
                    image_url=None,
                )
            )

        plans = crud_plan.batch_create(session=session, create_list=plan_create_list)
    return MultiGenerateResultsResponse(data=plans)
