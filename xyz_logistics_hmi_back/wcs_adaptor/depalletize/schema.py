# -*- coding: utf-8 -*-
from typing import List, Optional

import pydantic
from marshmallow import Schema, fields
from pydantic import BaseModel, Field

from apps.exceptions import XYZValidationError
from wcs_adaptor.entity import StrictSkuInfo
from wcs_adaptor.enums import TaskStatus, TaskType


class SkuInfoSchema(Schema):
    """SkuInfo的验证"""

    sku_id = fields.String(required=False)
    length = fields.Float(required=False)
    width = fields.Float(required=False)
    height = fields.Float(required=False)
    weight = fields.Float(required=False)
    sku_num = fields.Int(required=False, validate=lambda n: n > 0 or n == -1)
    sku_type = fields.String(required=False, load_from="type", dump_to="type")


class GetTaskInfoOutputSchema(BaseModel):
    """Rafcon获取任务的输出验证"""

    result: bool = Field(default=False, description="是否有可执行任务")
    task_id: Optional[str] = Field(default=None, description="任务编号")
    task_type: Optional[TaskType] = Field(default=None, description="任务类型")
    task_status: Optional[TaskStatus] = Field(default=None, description="任务状态")
    sku_info: Optional[StrictSkuInfo] = Field(default_factory=dict, description="物料尺寸信息")
    max_num: int = Field(default=0, description="任务最大执行数量")
    undone_num: int = Field(default=-1, description="未完成的数量")
    order_done_num: int = Field(default=0, description="订单已完成的数量")
    from_ws: str = Field(default="", description="抓取位ID", alias="from")
    to_ws: str = Field(default="", description="放置位ID", alias="to")
    clear_from_ws: bool = Field(default=False, description="抓取位是否已占清空")
    clear_to_ws: bool = Field(default=False, description="放置位是否已占清空")
    customized_data: dict = Field(default_factory=dict, description="自定义数据")
    pallet_tote_data: Optional[dict] = Field(default={},description="托盘料箱数据")
    pick_tote_data: Optional[dict] = Field(default={},description="抓取料箱数据")
    pallet_clear_list: list = Field(default=[], description="已更换的托盘")
    from_pallet_tote_data: Optional[dict] = Field(default={},description="合托任务拼托副（被拆）数据")
    to_pallet_tote_data: Optional[dict] = Field(default={},description="合托任务拼托主（码）工作空间数据")
    lower_layer: bool = Field(description="是否降层",default=False)
    lower_speed: bool = Field(description="是否降速",default=False)
        
class GetSingleClassTaskInfoSchema(Schema):
    """WCS获取单拆单码任务信息"""

    task_id = fields.String(required=True)
    sku_info = fields.Nested(SkuInfoSchema, required=True)
    from_ws = fields.String(equired=True, load_from="from", dump_to="from")
    to_ws = fields.String(required=True, load_from="to", dump_to="to")


class TaskCreateSchema(BaseModel):
    task_id: str = Field(description="任务ID")
    from_ws: str = Field(description="抓取位ID", alias="from")
    to_ws: str = Field(default="",description="放置位ID", alias="to")
    sku_type: int = Field(default=0,description="料箱类型")
    target_num: Optional[int] = Field(default=None, description="任务数量")
    pallet_tote_data: Optional[dict] = Field(default={},description="托盘料箱数据")
    pick_tote_data: Optional[dict] = Field(default={},description="拣配任务抓取料箱数据")
    pallet_clear_list: list = Field(default=[], description="拣配任务已更换的托盘")
    from_pallet_tote_data: Optional[dict] = Field(default={},description="合托任务拼托副（被拆）数据")
    to_pallet_tote_data: Optional[dict] = Field(default={},description="合托任务拼托主（码）工作空间数据")
    lower_layer: bool = Field(description="是否降层",default=False)
    lower_speed: bool = Field(description="是否降速",default=False)
    customized_data: Optional[dict] = Field(default={},description="自定义数据")
        
    @pydantic.validator("target_num")
    def validate_target_num(cls, num: int) -> int:
        """验证target_num是否有效.

        Args:
            num: 任务数量.

        Returns:
            int: 任务数量.
        """
        if num == 0:
            raise XYZValidationError(error_message="target_num can't equal 0.")
        if num < -1:
            raise XYZValidationError(error_message="target_num can't less than -1.")
        return num

    class Config:
        schema_extra = {"example": {"task_id": "124", "from": "0", "to": "1"}}


class SingleTaskCreateSchema(TaskCreateSchema):
    sku_info: StrictSkuInfo = Field(description="SKU信息",default={
                    "sku_id": "",
                    "length": 0,
                    "width": 0,
                    "height": 0,
                    "weight": 0,
                    "sku_num": 0,
                })

    class Config:
        schema_extra = {
            "example": {
                "task_id": "3",
                "sku_info": {
                    "sku_id": "",
                    "length": 1000,
                    "width": 1000,
                    "height": 1000,
                    "weight": 5,
                    "sku_num": 5,
                },
                "from": "1",
                "to": "2",
            }
        }


class OrderCreateSchema(BaseModel):
    order_id: str = Field(description="订单ID")
    from_ws: str = Field(description="抓取位ID", alias="from")
    to_ws: str = Field(description="放置位ID", alias="to")
    sku_info: List[StrictSkuInfo] = Field(default_factory=list, description="多个SKU信息")
    sync: Optional[bool] = Field(default=False, description="是否同步执行")

    class Config:
        schema_extra = {
            "example": {
                "order_id": "1",
                "sku_info": [
                    {
                        "sku_id": "1",
                        "length": 1000,
                        "width": 1000,
                        "height": 1000,
                        "weight": 5,
                        "sku_num": 5,
                    },
                    {
                        "sku_id": "2",
                        "length": 1000,
                        "width": 1000,
                        "height": 1000,
                        "weight": 5,
                        "sku_num": 5,
                    },
                ],
                "from": "1",
                "to": "2",
            }
        }


class GetMultiDepalTaskInfoSchema(Schema):
    """WCS获取混拆任务信息"""

    task_id = fields.String(required=True)
    from_ws = fields.String(required=True, load_from="from", dump_to="from")
    to_ws = fields.String(required=True, load_from="to", dump_to="to")


class GetMultiPalTaskInfoSchema(Schema):
    """WCS获取混码任务信息"""

    task_id = fields.String(required=True)
    sku_info = fields.List(fields.Nested(SkuInfoSchema), required=True)
    from_ws = fields.String(required=True, load_from="from", dump_to="from")
    to_ws = fields.String(required=True, load_from="to", dump_to="to")


class GetMultiPalStartInfoSchema(BaseModel):
    """WCS获取混码开始信息"""
    order_id: str
    round_id: int = Field(ge=0, description="混码轮次")

    class Config:
        schema_extra = {"example": {"order_id": "1", "round_id": 0}}


class IsWsReadyInputSchema(Schema):
    """Rafcon及WCS获取工作空间就位的输入验证"""

    ws_id: str = fields.String(required=True)
    pallet_id: Optional[str] = fields.String(required=False)  # 客户存储托盘编号
    agv_status:str = fields.String(required=False)


class IsWsReadyOutputSchema(Schema):
    """Rafcon及WCS获取工作空间就位的输出验证"""

    result = fields.Bool(required=True)
    error = fields.Int(required=True)
    error_message = fields.String(required=True)


class ReportTaskStatusInputSchema(BaseModel):
    """Rafcon回报任务状态输入验证"""

    task_id: str
    pick_num: int
    drop_num: Optional[int] = Field(ge=0, title="掉落数量")
    error: int
    error_message: str
    is_depal_pallet_empty: bool
    is_pal_pallet_full: bool
    pick_id: Optional[str] = Field(default="", description="单次抓取id")
    place_id:Optional[str] = Field(default="", description="单次放置id")
    pick_tote_data: Optional[dict] = Field(default={},description="单次抓取料箱数据")
    place_tote_data: Optional[dict] = Field(default={},description="单次放置料箱数据")


class ReportTaskEndingInputSchema(BaseModel):
    """Rafcon回报任务结束输入验证"""

    task_id: str
    error:int = Field(default=0,description="异常")

    class Config:
        schema_extra = {"example": {"task_id": "123456"}}


class ReportTaskStatusOutputSchema(Schema):
    """Rafcon回报任务状态输出验证"""

    error = fields.Int(required=True)
    error_message = fields.String(required=True)


class ErrorHandleInputSchema(Schema):
    """Rafcon回报异常输入验证"""

    error = fields.String(required=True)
    data = fields.Dict(default={}, required=True)

    class Config:
        schema_extra = {
            "example": {
                "error": "0",
                "data": {
                    "error_msg": "Start task failure",
                    "zh_msg": "任务开始失败",
                    "error_code": "E0000",
                    "tip": "请联系XYZ开发人员",
                },
            }
        }


class GetSkuInfoInputSchema(Schema):
    """XYZ 向 WCS请求SKU 信息 (扫码专用)输入验证"""

    sku_id = fields.String(required=True)
    customized_request = fields.Dict(required=True)

    class Config:
        schema_extra = {"example": {"sku_id": "xx", "customized_request": {}}}


class GetSkuInfoOutputSchema(Schema):
    """XYZ 向 WCS请求SKU 信息 (扫码专用)输出验证"""

    sku_info = fields.Nested(SkuInfoSchema, required=True)
    error = fields.Int(required=True)
    error_message = fields.String(required=True)


class ManualAddBoxInputSchema(Schema):
    pick_num: int = fields.Int(required=True, validate=lambda x: x >= 1)

    class Config:
        schema_extra = {"example": {"pick_num": 1}}


class UpdatePalletInput(Schema):
    sku_info = fields.Nested(SkuInfoSchema, required=True)
    error = fields.Int(required=True)
    error_message = fields.String(required=True)
