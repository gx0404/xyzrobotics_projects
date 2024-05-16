import sqlalchemy as sa

from apps.base_app.views.routes.entity import RouteEntity
from apps.enums import ProjectType
from apps.models.base import DBModelMixin, db


class RouteDBModel(db.Model, DBModelMixin["RoutesDBModel", RouteEntity]):
    __tablename__ = "routes"
    id = sa.Column(sa.Integer, primary_key=True, autoincrement=True)
    path = sa.Column(sa.String(50), nullable=False, index=True, comment="路径")
    project_type = sa.Column(
        sa.Enum(ProjectType),
        nullable=True,
        comment="项目类型"
    )
    name = sa.Column(sa.String(20), nullable=False, comment="名称")
