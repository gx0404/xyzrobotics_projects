#  -*- coding: utf-8 -*-
#  Copyright (c) XYZ Robotics Inc. - All Rights Reserved
#  Unauthorized copying of this file, via any medium is strictly prohibited
#  Proprietary and confidential
#  Author: Yuhang Wu <yuhang.wu@xyzrobotics.ai>, 2022/10/21 下午3:46
import sqlalchemy as sa
from sqlalchemy import text

from apps.models import db

from .enums import PlanningStatus


class PlanningResultModel(db.Model):
    """规划结果存储表"""
    __tablename__ = "planning_result"
    id = sa.Column(sa.Integer, primary_key=True, nullable=False)
    order_id = sa.Column(sa.String(100), nullable=False, comment="订单ID")
    status = sa.Column(sa.Enum(PlanningStatus), default=PlanningStatus.PENDING, nullable=False, comment="规划状态")
    result = sa.Column(sa.JSON, nullable=True, comment="规划结果")
    start_time = sa.Column(sa.DateTime, comment="开始规划的时间")
    end_time = sa.Column(sa.DateTime, comment="结束的时间")
    create_time = sa.Column(sa.DateTime, server_default=text('CURRENT_TIMESTAMP'), nullable=False, comment="创建时间")
    update_time = sa.Column(sa.DateTime, server_default=text('CURRENT_TIMESTAMP ON UPDATE CURRENT_TIMESTAMP'), nullable=False, comment="更新时间")
    is_deleted = sa.Column(sa.BOOLEAN, default=False, comment="是否已删除")

    __table_args__ = (
        sa.UniqueConstraint(
            "order_id",
            "is_deleted",
            name="uniq_order_is_del"
        ),
    )
