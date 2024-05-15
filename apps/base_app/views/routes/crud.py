from typing import List, Optional

from sqlalchemy import or_

from apps.base_app.views.routes.schema import (
    RouteQuerySchema,
    RouteUpdateSchema
)
from apps.exceptions import XYZNotFoundError
from apps.models import db
from apps.models.routes import RouteDBModel
from .entity import RouteEntity


class CRUDRoute:

    def filter_by_project_type(
        self,
        query: RouteQuerySchema
    ) -> List[RouteEntity]:
        """根据项目类型获取路由.

        Args:
            query: 查询类

        Returns:
            一批RouteEntity对象.
        """
        queryset: Optional[List[RouteDBModel]] = db.session.query(
            RouteDBModel
        ).filter(
            or_(
                RouteDBModel.project_type == query.project_type,
                RouteDBModel.project_type == None
            )
        )
        if queryset:
            return [RouteEntity.parse_obj(q.to_dict()) for q in queryset]
        else:
            return []

    def update(self, update: RouteUpdateSchema) -> RouteEntity:
        """编辑Route记录

        Args:
            update: 更新类.

        Returns:
            Route: 一个RouteEntity对象.

        Raises:
            XYZNotFoundError: 记录不存在.
        """
        route: RouteDBModel = db.session.query(RouteDBModel).filter(
            RouteDBModel.id == update.id
        ).first()
        if not route:
            raise XYZNotFoundError(
                error_message=f"ID为{update.id}的路由记录不存在."
            )
        route.name = update.name
        db.session.commit()
        return RouteEntity(
            id=route.id,
            path=route.path,
            name=route.name,
            project_type=route.project_type,
        )


crud_route = CRUDRoute()
