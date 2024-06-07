#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
    wcs_adaptor.piece_picking.schema
    ~~~~~~~~~~~~~~~~~~~~~

    包含对接口请求响应数据的格式校验及默认化设置

    Copyright (c) XYZ Robotics Inc. - All Rights Reserved
    Unauthorized copying of this file, via any medium is strictly prohibited
    Proprietary and confidential
    Developer: Chen Kun <kun.chen@xyzrobotics.ai>, May, 2022
"""
from marshmallow import Schema, fields, validates, ValidationError, pre_load

from wcs_adaptor.manager import task_manager
from wcs_adaptor.enums import TaskType

class NoticeWsSchema(Schema):
    ws_id = fields.String(required=True)

    class Config:
        schema_extra = {
            "example": {
                "ws_id": "ws_1"
            }
        }


class SkuInfoSchema(Schema):
    """返回给XTF的SKU物料信息校验"""
    sku_id = fields.String()
    length = fields.Float()
    width = fields.Float()
    height = fields.Float()
    weight = fields.Float()
    sku_type = fields.String()


class StandardTaskSchema(Schema):
    """根据标准的任务报文字段格式进行校验

    Examples:

        {
            "picking_position": "1",
            "picking_bin": "0",
            "placing_position": "3",
            "placing_bin": "0",
            "sku_info": {
                "sku_id": "001",
                "length": 100.00,
                "width": 100.00,
                "height": 100.00,
                "weight": 200.00,
                "sku_type": "box"
            },
            "target_num": 10
        }
    """
    task_type = fields.Integer(load_default=100)    # 2.0.0版本无法使用load_default参数
    picking_code = fields.String(required=False, allow_none=True)
    picking_position = fields.String(required=False, allow_none=True)
    picking_bin = fields.String(required=False, allow_none=True)
    placing_code = fields.String(required=False, allow_none=True)
    placing_position = fields.String(required=False, allow_none=True)
    placing_bin = fields.String(required=False, allow_none=True)
    sku_info = fields.Nested(SkuInfoSchema, required=False, allow_none=True)
    # 任务数量必须要传，如果任务数大于
    target_num = fields.Integer(required=True)

    @pre_load
    def set_task_type_default(self, input: dict) -> None:
        input.setdefault("task_type", 100)

    @validates('target_num')
    def validate_target_num(self, value):
        if value < -1:
            raise ValidationError('任务数错误，数量必须大于等于-1，且为整数')

    @validates('task_type')
    def validate_task_type(self, value):
        if value < 100 or value >= 200:
            raise ValidationError('任务类型错误，必须为100-199的整数')
        if value not in TaskType.__members__.values():
            raise ValidationError("任务类型不存在于小件拣选任务类型中")


class StandardOrderSchema(Schema):
    """根据标准的订单报文字段格式进行校验

    Examples:

        {
            "order_id": "0001",
            "tasks": [
                {
                    "picking_position": "1",
                    "picking_bin": "0",
                    "placing_position": "3",
                    "placing_bin": "0",
                    "sku_info": {
                        "sku_id": "001",
                        "length": 100,
                        "width": 100,
                        "height": 100,
                        "weight": 200,
                        "sku_type": "box"
                    },
                    "target_num": 10
                }
            ]
        }
    """
    order_id = fields.String(required=True)
    task_type = fields.Integer(load_from=100)
    tasks = fields.Nested(StandardTaskSchema, many=True)

    class Config:
        schema_extra = {
            "example": {
                "order_id": "0001",
                "tasks": [
                    {
                        "picking_position": "1",
                        "picking_bin": "0",
                        "placing_position": "3",
                        "placing_bin": "0",
                        "sku_info": {
                            "sku_id": "001",
                            "length": 100,
                            "width": 100,
                            "height": 100,
                            "weight": 200,
                            "sku_type": "box"
                        },
                        "target_num": 10
                    }
                ]
            }
        }


class GetTaskInfoOutputSchema(Schema):
    """XTF获取任务信息校验"""
    error = fields.Integer()
    error_message = fields.String()
    result = fields.Boolean()
    # 分拣任务的具体信息
    task_id = fields.String()
    sku_info = fields.Nested(SkuInfoSchema)
    num_to_pick = fields.Int()
    pick_code = fields.String()
    pick_ws = fields.String()
    place_code = fields.String()
    place_ws = fields.String()
    return_ws = fields.String()


class ReportStepOutcomeSchema(Schema):
    """XTF反馈分拣结果(包含正常分拣、空桶、满眶）"""
    task_id = fields.String(required=True)
    error = fields.Integer(required=True)
    error_message = fields.String(required=True)
    place_ws = fields.String()

    @validates('task_id')
    def validate_task_id(self, value):
        if task_manager.get_task_by_id(value) is None:
            raise ValidationError(f'Unknown task id {value}')


class ErrorHandleInputSchema(Schema):
    """XTF回报异常输入验证"""
    error = fields.String()
    data = fields.Dict(allow_none=True)


class RobotMotionSchema(Schema):
    """是否启用xtf接口"""
    get_task_info = fields.Bool(required=False)
    allow_pick = fields.Bool(required=False)
    allow_move = fields.Bool(required=False)
    allow_release = fields.Bool(required=False)
    report_step_outcome = fields.Bool(required=False)

    class Config:
        schema_extra = {
            "example": {
                "get_task_info": True,
                "allow_pick": True,
                "allow_move": True,
                "allow_release": True,
                "report_step_outcome": True
            }
        }


