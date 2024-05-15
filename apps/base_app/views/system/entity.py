"""存放系统相关的实体"""
from typing import Optional

from pydantic import BaseModel, Field

from apps.settings import Language


class LogoEntity(BaseModel):
    """Logo实体."""
    logo: str = Field(default=None, description="logo图片base64")


class SystemEntity(BaseModel):
    """系统信息实体."""
    logo: Optional[str] = Field(default=None, description="logo图片, base64")
    version: str = Field(description="系统版本")
    language: Language = Field(description="系统语言")
