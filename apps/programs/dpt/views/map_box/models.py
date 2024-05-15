# -*- coding: utf-8 -*-
"""
Copyright (c) XYZ Robotics Inc. - All Rights Reserved
Unauthorized copying of this file, via any medium is strictly prohibited
Proprietary and confidential
Author: Yuhang Wu <yuhang.wu@xyzrobotics.ai>, 2022-04-14
"""
from datetime import datetime

import sqlalchemy as sa
from sqlalchemy import Column

from apps.models import db
from apps.models.base import DBModelMixin
from apps.programs.dpt.views.map_box.entity import MapBoxEntity


class MapBoxDBModel(db.Model, DBModelMixin["MapBoxDBModel", MapBoxEntity]):
    __tablename__ = "dpt_register_box"
    id = Column("id", sa.Integer, primary_key=True, autoincrement=True)
    serial_number = Column(sa.String(50), nullable=False, comment="纸箱编号")
    normal_length = sa.Column(sa.Numeric(10, 2), nullable=False, comment="标准长度")
    normal_width = sa.Column(sa.Numeric(10, 2), nullable=False, comment="标准宽度")
    normal_height = sa.Column(sa.Numeric(10, 2), nullable=False, comment="标准高度")
    real_length = sa.Column(sa.Numeric(10, 2), nullable=False, comment="实际长度")
    real_width = sa.Column(sa.Numeric(10, 2), nullable=False, comment="实际宽度")
    real_height = sa.Column(sa.Numeric(10, 2), nullable=False, comment="实际高度")
    # V1.2.0版本之前没有normal_weight和real_weight, 为了兼容性, 这两个字段允许为空.
    normal_weight = sa.Column(
        sa.Numeric(10, 2),
        nullable=True,
        comment="标准重量"
    )
    real_weight = sa.Column(
        sa.Numeric(10, 2),
        nullable=True,
        comment="实际重量"
    )
    extra1 = sa.Column(sa.String(255), nullable=True, comment="备用字段1")
    extra2 = sa.Column(sa.String(255), nullable=True, comment="备用字段2")
    extra3 = sa.Column(sa.String(255), nullable=True, comment="备用字段3")
    extra4 = sa.Column(sa.String(255), nullable=True, comment="备用字段4")
    create_time = sa.Column(
        sa.DateTime,
        index=True,
        nullable=False,
        default=lambda: datetime.now(),
        comment="创建时间"
    )
    update_time = sa.Column(
        sa.DateTime,
        index=True,
        nullable=False,
        default=lambda: datetime.now(),
        onupdate=lambda: datetime.now(),
        comment="更新时间"
    )
    is_del = sa.Column(sa.BOOLEAN, default=False, comment="是否已删除")

    __table_args__ = (
        sa.UniqueConstraint(
            "serial_number",
            "is_del",
            name="uniq_serial_number_is_del"
        ),
        sa.UniqueConstraint(
            "normal_length",
            "normal_width",
            "normal_height",
            "is_del",
            name="uniq_normal_l_w_h"
        )
    )
