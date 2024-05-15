# -*- coding: utf-8 -*-
"""
Copyright (c) XYZ Robotics Inc. - All Rights Reserved
Unauthorized copying of this file, via any medium is strictly prohibited
Proprietary and confidential
Author: Yuhang Wu <yuhang.wu@xyzrobotics.ai>, 2022-07-22
"""
import json
from datetime import datetime
from typing import Any, Optional

from pydantic import BaseModel, Field, validator, root_validator

from apps.enums import ErrorSources


class ErrorRecordsEntity(BaseModel):
    id: int
    task_id: Optional[str] = Field(default=None, title="任务编号")
    task: Optional[dict] = Field(default=None, title="任务快照")
    error_code: str = Field(default=None, title="异常码")
    error_msg: str = Field(default=None, title="异常消息")
    error_source: ErrorSources = Field(title="异常来源")
    tip: str = Field(default=None, title="异常提示")
    create_time: Optional[datetime] = Field(title="异常发生的时间")

    class Config:
        orm_mode = True

    @validator("task", pre=True)
    def validate_task(cls, v: Any) -> Optional[dict]:
        """验证 task 的类型.

        从 SQLAlchemy 拿到的 JSON 类型，其实是 dict，在此处将其转换 Json.
        """
        return v if isinstance(v, dict) or v is None else json.loads(v)

    @root_validator()
    def root_validate(cls, values: dict) -> dict:
        if task := values.get("task", None):
            values["task_id"] = task.get("task_id", None)
        return values
