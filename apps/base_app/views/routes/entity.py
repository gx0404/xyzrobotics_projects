# -*- coding: utf-8 -*-
"""
Copyright (c) XYZ Robotics Inc. - All Rights Reserved
Unauthorized copying of this file, via any medium is strictly prohibited
Proprietary and confidential
Author: Yuhang Wu <yuhang.wu@xyzrobotics.ai>, 2022-07-18
"""
from typing import Optional

from pydantic import BaseModel, Field

from apps.enums import ProjectType


class RouteEntity(BaseModel):
    route_id: int = Field(alias="id")
    to: str = Field(alias="path")
    project_type: Optional[ProjectType] = None
    name: str = Field(title="名称")
