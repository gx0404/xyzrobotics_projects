#  -*- coding: utf-8 -*-
#  Copyright (c) XYZ Robotics Inc. - All Rights Reserved
#  Unauthorized copying of this file, via any medium is strictly prohibited
#  Proprietary and confidential
#  Author: Yuhang Wu <yuhang.wu@xyzrobotics.ai>, 2022/10/22 下午2:19
from datetime import datetime
from typing import Optional

from pydantic import BaseModel

from apps.ext.offline_mix_planner.entity import PlanningResult, PlanningResultRecord
from apps.ext.offline_mix_planner.enums import PlanningStatus
from apps.schemas import SingleResponseSchema


class CreateSchema(BaseModel):
    order_id: str
    status: PlanningStatus = PlanningStatus.PENDING
    result: Optional[PlanningResult] = None
    create_time: Optional[datetime] = None


class UpdateSchema(BaseModel):
    status: Optional[PlanningStatus] = None
    result: Optional[PlanningResult] = None
    start_time: Optional[datetime] = None
    end_time: Optional[datetime] = None
    is_deleted: Optional[bool] = False


class Response(SingleResponseSchema[PlanningResultRecord]):
    pass

