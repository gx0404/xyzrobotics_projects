from datetime import datetime
from decimal import Decimal
from typing import Dict, Optional, Any, Literal

from pydantic import BaseModel, Field, validator

from apps.exceptions import XYZValidationError
from apps.settings import settings


class ExtraColumn(BaseModel):
    """额外的列"""
    key: str = Field(description="键")
    value: Any = Field(None, description="值")


class MapBoxEntity(BaseModel):
    """纸箱实体"""
    serial_number: str = Field(title="纸箱编号", description="纸箱编号")
    normal_length: Decimal = Field(title="标准长度", description="标准长度", gt=0, le=20000)
    normal_width: Decimal = Field(title="标准宽度", description="标准宽度", gt=0, le=20000)
    normal_height: Decimal = Field(title="标准高度", description="标准高度", gt=0, le=20000)
    normal_weight: Decimal = Field(title="标准重量", description="标准重量", gt=0, le=10000)
    real_length: Decimal = Field(title="实际长度", description="实际长度", gt=0, le=20000)
    real_width: Decimal = Field(title="实际宽度", description="实际宽度", gt=0, le=20000)
    real_height: Decimal = Field(title="实际高度", description="实际高度", gt=0, le=20000)
    real_weight: Decimal = Field(title="实际重量", description="实际重量", gt=0, le=10000)
    extra1: Optional[ExtraColumn] = Field(default=None, title="扩展字段1", description="扩展字段1")
    extra2: Optional[ExtraColumn] = Field(default=None, title="扩展字段2", description="扩展字段2")
    extra3: Optional[ExtraColumn] = Field(default=None, title="扩展字段3", description="扩展字段3")
    extra4: Optional[ExtraColumn] = Field(default=None, title="扩展字段4", description="扩展字段4")
    extra: Dict[str, Any] = Field(default_factory=dict, title="扩展字段集合", description="扩展字段集合，集中extra1～4")
    create_time: Optional[datetime] = Field(default=None, description="创建时间")
    update_time: Optional[datetime] = Field(default=None, description="更新时间")

    class Config:
        orm_mode = True

    @staticmethod
    def _check_extra_field(
        name: Literal["extra1", "extra2", "extra3", "extra4"],
        v: Optional[str]
    ) -> Optional[ExtraColumn]:
        """检查额外字段"""
        if v is None:
            return v
        if settings.dpt_settings and settings.dpt_settings.extra_columns_for_register_box:
            column = settings.dpt_settings.extra_columns_for_register_box.get(name, None)
        else:
            column = None
        if column is None:
            raise XYZValidationError(f"'{name}' 字段未配置")
        if not column.enable:
            return None
        if isinstance(v, ExtraColumn):
            return v
        elif isinstance(v, str):
            return ExtraColumn(key=column.key or name, value=v)
        else:
            raise XYZValidationError(f"'{name}' 字段类型错误, 应为 str 或 ExtraColumn")

    @validator("extra1", pre=True)
    def extra1_validator(cls, v) -> Optional[ExtraColumn]:
        """额外字段1"""
        return cls._check_extra_field("extra1", v)

    @validator("extra2", pre=True)
    def extra2_validator(cls, v) -> Optional[ExtraColumn]:
        """额外字段2"""
        return cls._check_extra_field("extra2", v)

    @validator("extra3", pre=True)
    def extra3_validator(cls, v) -> Optional[ExtraColumn]:
        """额外字段3"""
        return cls._check_extra_field("extra3", v)

    @validator("extra4", pre=True)
    def extra4_validator(cls, v) -> Optional[ExtraColumn]:
        """额外字段4"""
        return cls._check_extra_field("extra4", v)

    @validator("extra")
    def fill_extra(cls, v: dict, values: dict) -> dict:
        """填充额外字段"""
        for name in ["extra1", "extra2", "extra3", "extra4"]:
            column = settings.dpt_settings.extra_columns_for_register_box.get(name, None)  # type: ignore
            if column is None or not column.enable:
                continue
            key = column.key or name
            if name not in values:
                continue
            v[key] = values[name] and values[name].value
        return v
