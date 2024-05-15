import sqlalchemy as sa

from apps.base_app.views.error_records.entity import ErrorRecordsEntity
from apps.enums import ErrorSources
from apps.models import db
from apps.models.base import DBModelMixin


class ErrorRecordsDBModel(
    db.Model,
    DBModelMixin["ErrorRecordsDBModel", ErrorRecordsEntity]
):
    __tablename__ = "error_records"
    id = sa.Column(sa.Integer, primary_key=True, autoincrement=True)
    error_code = sa.Column(sa.String(20), nullable=True, comment="异常状态码")
    error_msg = sa.Column(sa.Text, nullable=True, comment="异常消息")
    error_source = sa.Column(sa.Enum(ErrorSources), comment="异常来源")
    tip = sa.Column(sa.Text, nullable=True, comment="异常提示")
    task = sa.Column(sa.JSON, nullable=True, comment="任务快照")
    create_time = sa.Column(
        sa.DateTime,
        server_default="now()",
        index=True,
        comment="创建时间"
    )
