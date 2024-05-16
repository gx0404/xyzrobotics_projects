from datetime import datetime
from typing import (
    Optional
)

from pydantic import BaseModel, Field

from wcs_adaptor.entity import SkuInfo
from wcs_adaptor.enums import TaskStatus, TaskType


class TaskHistory(BaseModel):
    id: int = Field(description="主键ID")
    task_id: str = Field(description="任务ID")
    order_id: Optional[str] = Field(default=None, description="订单ID")
    task_type: TaskType = Field(description="任务类型")
    task_status: TaskStatus = Field(description="任务状态")
    sku_info: Optional[SkuInfo] = Field(default=None, description="SKU物料信息")
    target_num: int = Field(description="预计需要完成的数量")
    done_num: int = Field(description="已完成的数量")
    from_ws: str = Field(description="抓取位工作空间号")
    to_ws: str = Field(description="放置位工作空间号")
    customized_data: dict = Field(default_factory=dict, description="自定义数据")
    start_time: Optional[datetime] = Field(default=None, description="任务开始时间")
    end_time: Optional[datetime] = Field(default=None, description="任务结束时间")
    create_time: datetime = Field(description="任务创建时间")
    error_code: Optional[int] = Field(description="异常码")
    error_msg: Optional[str] = Field(description="异常详细x")
