from datetime import datetime

import sqlalchemy as sa
from sqlalchemy import (
    Boolean,
    Column,
    ForeignKey,
    Integer,
    JSON,
    Numeric,
    String,
    Text,
    Unicode
)
from sqlalchemy.orm import relationship

from apps.models import db


def to_dict(self):
    return {c.name: getattr(self, c.name, None) for c in self.__table__.columns}


# db.Model.to_dict = to_dict
db.Model.__table_args__ = {
    "mysql_charset": "utf8"
}


class Pallet(db.Model):
    # table name
    __tablename__ = 'pallet'
    # columns
    id = Column(Integer, primary_key=True)
    name = Column(Unicode(64), nullable=False, comment="托盘名称")
    length = Column(Numeric(precision=10, scale=2))
    width = Column(Numeric(precision=10, scale=2))
    height = Column(Numeric(precision=10, scale=2))
    max_height = Column(Numeric(precision=10, scale=2))
    max_weight = Column(Numeric(precision=10, scale=2))
    # TODO(YuhangWu): 之后版本更改其类型, 不应该使用布尔类型
    pallet_type = Column(Boolean)
    thickness = Column(Numeric(precision=10, scale=2))
    design = relationship("Design", uselist=True, back_populates=__tablename__)
    is_del = sa.Column(sa.BOOLEAN, default=False, comment="是否已删除")

    __table_args__ = (
        sa.UniqueConstraint(
            "name",
            "is_del",
            name="uniq_name_is_del"
        ),
    )

    def __repr__(self):
        return '<Pallet:{} + {}>'.format(self.name, self.id)


class Box(db.Model):
    # table name
    __tablename__ = 'box'
    # columns
    id = Column(Integer, primary_key=True)
    name = Column(Unicode(64), nullable=False, comment="箱子名称")
    length = Column(Numeric(precision=10, scale=2))
    width = Column(Numeric(precision=10, scale=2))
    height = Column(Numeric(precision=10, scale=2))
    weight = Column(Numeric(precision=10, scale=2))
    scan_code = Column(String(64), nullable=False, comment="条码")
    img_url = Column(Text)
    batch_no = Column(String(32), comment="批次号")
    design = relationship("Design", uselist=False, back_populates=__tablename__)
    is_del = sa.Column(sa.BOOLEAN, default=False, comment="是否已删除")

    __table_args__ = (
        sa.UniqueConstraint(
            "name",
            "is_del",
            name="uniq_name_is_del"
        ),
        sa.UniqueConstraint(
            "scan_code",
            "is_del",
            name="uniq_scan_code_is_del"
        ),
    )

    def __repr__(self):
        return '<Box:{} + {}>'.format(self.name, self.id)


class Design(db.Model):
    __tablename__ = 'design'
    id = Column(Integer, primary_key=True)
    # name 字段没要用到
    name = Column(Unicode(64))
    batch_no = Column(String(32), comment="批次号")
    # 与托盘的关系，多对一
    pallet_id = Column(Integer, ForeignKey('pallet.id'), nullable=False)
    # 与托盘的关系，一对一
    box_id = Column(Integer, ForeignKey('box.id'), nullable=False)
    pallet = relationship("Pallet", back_populates="design")
    box = relationship("Box", back_populates="design")
    guillotine_packing = Column(Integer)
    barcode_direction = Column(Integer)
    mirror = Column(Integer)
    flip = Column(String(10))
    layers = Column(Integer, comment="层数")
    layout = Column(JSON, comment="规划数据")
    objects = Column(JSON, comment="规划数据")
    image_url = Column(String(100), comment="垛型图片链接")
    create_time = Column(sa.DateTime, default=datetime.now, server_default=sa.text("NOW()"), comment="创建时间")
    update_time = Column(sa.DateTime, onupdate=sa.text("NOW()"), comment="上次更新时间")
    is_del = sa.Column(sa.BOOLEAN, default=False, comment="是否已删除")

    __table_args__ = (
        sa.UniqueConstraint(
            "name",
            "is_del",
            name="uniq_name_is_del"
        ),
    )

