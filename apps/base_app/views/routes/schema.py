from typing import List

from pydantic import BaseModel, Field

from apps.enums import ProjectType
from apps.schemas import Response as BaseResponse
from apps.settings import settings
from .entity import RouteEntity


class RouteQuerySchema(BaseModel):
    project_type: ProjectType = settings.PROJECT_TYPE.upper()


class RouteUpdateSchema(BaseModel):
    id: int = Field(alias="route_id")
    name: str = Field(title="名称", max_length=20, min_length=1)


class Routes(BaseModel):
    __root__: List[RouteEntity]


class Response(BaseResponse):
    data: RouteEntity


class ListResponse(BaseResponse):
    data: Routes
