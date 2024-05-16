# -*- coding: utf-8 -*-
"""
Copyright (c) XYZ Robotics Inc. - All Rights Reserved
Unauthorized copying of this file, via any medium is strictly prohibited
Proprietary and confidential
Author: Yuhang Wu <yuhang.wu@xyzrobotics.ai>, 2022-07-22

对XTF回报的一些异常进行记录.
"""
from typing import Any
from apps.base_app.crud import CRUDBase
from apps.base_app.views.error_records.entity import ErrorRecordsEntity
from apps.base_app.views.error_records.schemas import ErrorRecordsCreateSchema
from apps.models.error_records import ErrorRecordsDBModel


class CRUDErrorRecords(
    CRUDBase[
        ErrorRecordsDBModel, ErrorRecordsEntity, Any, ErrorRecordsCreateSchema, Any
    ]
):
    pass


crud_error_records = CRUDErrorRecords(
    model=ErrorRecordsDBModel, entity=ErrorRecordsEntity
)
