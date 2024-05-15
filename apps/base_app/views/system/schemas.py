from pydantic import BaseModel

from apps.settings import Language
from apps.schemas import GetResponseSchema
from apps.base_app.views.system.entity import LogoEntity, SystemEntity


class UpdateLanguageSchema(BaseModel):
    """用于更新系统语言"""
    language: Language


class SystemOuterSchema(SystemEntity):
    """向外部展示的模型"""
    pass


class SystemInfoResponse(GetResponseSchema[SystemOuterSchema]):
    pass


class LogoResponse(GetResponseSchema[LogoEntity]):
    pass
