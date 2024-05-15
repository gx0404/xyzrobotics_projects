from typing import Generic, List, Optional, Protocol, Type, TypeVar

import sqlalchemy.exc
from flask_sqlalchemy.model import DefaultMeta
from pydantic import BaseModel
from sqlalchemy.orm import Query
from sqlalchemy.orm.scoping import ScopedSession

from apps.exceptions import XYZIntegrityError, XYZNotFoundError
from apps.models import db
from apps.schemas import QuerySchema


class UpdateSchemaProtocol(Protocol):
    id: int

    def dict(self, *args, **kwargs) -> dict:
        pass


CreateSchemaType = TypeVar("CreateSchemaType", bound=BaseModel)
UpdateSchemaType = TypeVar("UpdateSchemaType", bound=UpdateSchemaProtocol)
QuerySchemaType = TypeVar("QuerySchemaType", bound=QuerySchema)
EntityType = TypeVar("EntityType", bound=BaseModel)
DBModelType = TypeVar("DBModelType", bound=DefaultMeta)


class CRUDBase(
    Generic[
        DBModelType,
        EntityType,
        QuerySchemaType,
        CreateSchemaType,
        UpdateSchemaType
    ]
):

    def __init__(self, model: Type[DBModelType], entity: Type[EntityType]):
        """

        Args:
            model: 数据库模型类.
            entity: 实体类.
        """
        self.model = model
        self.entity = entity

    def _get_raw_model(
        self,
        session: ScopedSession = db.session,
        *,
        pk: int
    ) -> Optional[DBModelType]:
        """获取原始记录.

        Args:
            session(ScopedSession): 数据库会话对象.
            pk(int): 主键值.

        Returns:
            DBModelType or None: 返回数据库模型对象, 不存在则返回None.
        """
        return session.query(self.model).filter(self.model.id == pk).first()

    def get(
        self,
        session: ScopedSession = db.session,
        *,
        pk: int
    ) -> Optional[EntityType]:
        """获取一条记录.

        Args:
            session(ScopedSession): 数据库会话.
            pk(int): 主键值.

        Returns:
            Entity or None: 返回一个实体对象，不存在时返回None.
        """
        model = self._get_raw_model(session=session, pk=pk)
        if model:
            return self.entity.from_orm(model)

    def get_or_404(
        self,
        session: ScopedSession = db.session,
        *,
        pk: int
    ) -> EntityType:
        """根据主键值获取记录，不存在则抛出404异常.

        Args:
            session(ScopedSession): 会话对象.
            pk(int): 主键ID

        Returns:
            Entity: 一个实体对象.

        Raises:
            XYZNotFoundError: 记录不存在异常.
        """
        entity = self.get(session=session, pk=pk)
        if not entity:
            raise XYZNotFoundError(
                error_message=f"Retrieve failed, Not Found {self.entity.__name__}({pk})."
            )
        return entity

    def create(
        self,
        session: ScopedSession,
        create: CreateSchemaType
    ) -> EntityType:
        """创建一条记录.

        调用此方法后，并没有实际持久化到数据库.
        需要在调用处，主动commit提交，使该事务生效.

        Examples:
            from apps.models import db
            session = db.session
            crud_xxx.create(session, create=...)
            session.commit()

        Args:
            session(ScopedSession): 数据库会话.
            create: 用于创建的模式对象.

        Returns:
            PalletEntity: 返回一条已创建的实体对象.
        """
        instance: DBModelType = self.model(**create.dict())
        try:
            session.add(instance)
            session.flush()
        except sqlalchemy.exc.IntegrityError as err:
            session.rollback()
            raise XYZIntegrityError(
                error_message=f"{self.entity.__name__} already exists: {str(err.args)}"
            ) from err
        return self.entity.from_orm(instance)

    def _search(
        self,
        session: ScopedSession = db.session,
        *,
        query: Optional[QuerySchemaType] = None,
        is_filter: bool = True,
        is_collation: bool = True,
    ) -> Query:
        """搜索记录."""
        queryset: Query = session.query(self.model)
        if is_filter or is_collation:
            assert query is not None, "`is_filter`, `is_collation` 为 True 时，`query` 不能为 None."
        if is_filter:
            filters = query.to_sa_filters()
            queryset = queryset.filter(*filters)
        if is_collation:
            collation = query.to_sa_collation()
            queryset = queryset.order_by(*collation)
        return queryset

    def _pagination(
        self,
        session: ScopedSession = db.session,
        *,
        query: QuerySchemaType
    ) -> Query:
        """分页查询."""
        queryset = self._search(session, query=query)
        return queryset.limit(query.page_size).offset((query.page - 1) * query.page_size)

    def search(
        self,
        session: ScopedSession = db.session,
        *,
        query: QuerySchemaType,
        paginate: bool = True
    ) -> List[EntityType]:
        """搜索托盘记录.

        Args:
            session(ScopedSession): 数据库会话.
            query(QuerySchema): 用于搜索的模式对象.
            paginate(bool): 是否为分页查询, 默认为 True.

        Returns:
            List[Entity]: 返回一批实体对象.
        """
        if paginate:
            result = self._pagination(session=session, query=query)
        else:
            result = self._search(session=session, query=query)
        return [self.entity.from_orm(q) for q in result]

    def all(
        self,
        session: ScopedSession = db.session,
        *,
        query: QuerySchemaType
    ) -> List[EntityType]:
        """查询所有记录.

        Args:
            session(ScopedSession): 数据库会话.
            query(QuerySchema): 用于搜索的模式对象.

        Returns:
            List[Entity]: 返回一批实体对象.
        """
        return self.search(session=session, query=query, paginate=False)

    def count(
        self,
        session: ScopedSession = db.session,
        *,
        query: QuerySchemaType
    ) -> int:
        """统计纸箱总数.

        Args:
            session(ScopedSession): 数据库会话.

        Returns:
            int: 满足查询条件的总数.
        """
        filters = query.to_sa_filters()
        return session.query(self.model).filter(*filters).count()

    def pagination(
        self,
        session: ScopedSession = db.session,
        *,
        query: QuerySchemaType,
    ) -> dict:
        """分页查询"""
        return {
            "page": query.page,
            "page_size": query.page_size,
            "count": self.count(session=session, query=query),
            "data": self.search(session=session, query=query)
        }

    def update(
        self,
        session: ScopedSession,
        *,
        pk: Optional[int] = None,
        update: UpdateSchemaType
    ) -> EntityType:
        """整体更新一条记录.

        Args:
            session(ScopedSession): 会话对象.
            pk(int or None): 主键ID, 如果为None, 则从update取id.
            update(PalletUpdateSchema): 用于更新的模式对象.

        Returns:
            Entity: 一个实体对象.

        Raises:
            XYZNotFoundError: 记录不存在异常.
        """
        _id = pk or update.id
        model = self._get_raw_model(session=session, pk=_id)
        if not model:
            raise XYZNotFoundError(
                error_message=f"Update failed, Not Found {self.entity.__name__}({_id})."
            )
        for key, val in update.dict().items():
            setattr(model, key, val)
        session.flush()
        return self.entity.from_orm(model)

    # TODO(YuhangWu): 新增参数 `hard`, 用于指定物理删除和逻辑删除.
    def delete(
        self,
        session: ScopedSession,
        *,
        pk: int
    ) -> Optional[EntityType]:
        """根据主键值删除记录，不存在则返回None.

        Args:
            session(ScopedSession): 会话对象.
            pk(int): 主键ID

        Returns:
            Entity: 一个实体对象.

        Raises:
            XYZNotFoundError: 记录不存在异常.
        """
        model = self._get_raw_model(session=session, pk=pk)
        if model:
            session.delete(model)
            return self.entity.from_orm(model)

    # TODO(YuhangWu): 新增参数 `hard`, 用于指定物理删除和逻辑删除.
    def delete_or_404(
        self,
        session: ScopedSession,
        *,
        pk: int
    ) -> EntityType:
        """根据主键值删除记录，不存在则抛出404异常.

        Args:
            session(ScopedSession): 会话对象.
            pk(int): 主键ID

        Returns:
            Entity: 一个实体对象.

        Raises:
            XYZNotFoundError: 记录不存在异常.
        """
        if deleted_entity := self.delete(session=session, pk=pk):
            return deleted_entity
        else:
            raise XYZNotFoundError(
                error_message=f"Delete failed, Not Found {self.entity.__name__}({pk})."
            )

    def remove_all(
        self,
        session: ScopedSession,
        *,
        query: QuerySchemaType,
        hard: bool = False,
    ) -> List[EntityType]:
        """移除所有记录并返回记录.

        Args:
            session(ScopedSession): 会话对象.
            query(QuerySchema): 用于搜索的模式对象.
            hard(bool): 默认 False, 是否为物理删除

        Returns:
            List[EntityType]: 实体对象.
        """
        queryset = self._search(session=session, query=query, is_collation=False)
        results = [self.entity.from_orm(q) for q in queryset]
        self._delete(queryset, hard)
        return results

    def delete_all(
        self,
        session: ScopedSession,
        *,
        query: QuerySchemaType,
        hard: bool = False,
    ) -> int:
        """删除所有记录

        Args:
            session(ScopedSession): 会话对象.
            query(QuerySchema): 用于搜索的模式对象.
            hard(bool): 默认 False, 是否为物理删除

        Returns:
            int: 成功删除的行数.
        """
        queryset = self._search(session=session, query=query, is_collation=False)
        return self._delete(queryset, hard)

    def _delete(self, queryset: Query, hard: bool = False) -> int:
        """封装的删除方法, 用于支持逻辑删除和物理删除.

        Args:
            queryset(Query): SQLAlchemy.orm.Query.
            hard(bool): 默认 False, 是否为物理删除.

        Returns:
            int: 影响的行数.
        """
        if hard:
            return queryset.delete(False)
        elif hasattr(self.model, "is_del"):
            return queryset.update({self.model.is_del: None}, False)
        else:
            raise ValueError("没有 `is_del` 字段，无法进行逻辑删除.")
