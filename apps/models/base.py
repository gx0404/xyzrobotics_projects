from typing import Dict, Generic, List, Protocol, Type, TypeVar

from pydantic_sqlalchemy import sqlalchemy_to_pydantic

from apps.models import db


# 鸭子类型, 支持dict方法的类, 均可算作实体类
class EntityProto(Protocol):
    def dict(self) -> Dict:
        ...


EntityType = TypeVar("EntityType", bound=EntityProto)
DBModelType = TypeVar("DBModelType", bound=db.Model)


class DBModelMixin(Generic[DBModelType, EntityType]):
    """应用于扩展SQLAlchemy数据模型类的扩展类

    数据模型类, 继承此类即可.
    """

    def __init__(self, *args, **kwargs):
        super(DBModelMixin, self).__init__(*args, **kwargs)

    @classmethod
    def from_entity(cls, entity: EntityType) -> DBModelType:
        """通过实体对象转为用于SQLAlchemy操作的数据模型对象

        Args:
            entity(pydantic.BaseModel): an instance of BaseModel.

        Returns:

        """
        obj = cls()
        for key, val in entity.dict().items():
            setattr(obj, key, val)
        return obj

    @classmethod
    def to_total_entity_cls(cls) -> Type[EntityType]:
        """将数据模型类转为完整实体类

        此方法是转换得到的实体类与我们自定义实体类是有出入的.
        该实体类完全基于数据模型类构造, 其包含的属性对于数据表而言是完整的.
        该实体类包含的属性个数必定大于等于我们自定义的实体类.

        自定义实体类:
        class Entity(BaseModel):
            id: int
            name: str

        模型类:
        class DBModel(db.Model):
            id: int
            name: str
            is_del: bool

        上方"自定义实体类"与"模型类"相差一个 `is_del` 字段.
        当使用`DBModel.to_entity_cls()`所得到的实体类, 则会包含`is_del`字段.
        这是 **自定义实体类** 与 **完整实体类** 的区别.

        Returns:

        """
        return sqlalchemy_to_pydantic(cls)

    @property
    def entity(self) -> EntityType:
        """返回实体对象"""
        return self.to_entity()

    def to_entity(self, entity_cls: Type[EntityType] = None) -> EntityType:
        """根据实体类, 将数据模型类转为实体对象

        Args:
            entity_cls: 实体类

        Returns:

        """
        if entity_cls is None:
            return self.to_total_entity_cls().from_orm(self)
        return entity_cls(**self.to_dict())

    @staticmethod
    def to_entities(query_set: List[DBModelType]) -> List[EntityType]:
        """将一批 ``SQLAlchemy`` 的查询结果转为实体对象集

        Args:
            query_set: SQLAlchemy查询方法返回的对象
                例如: query_set = db.session.query(Model).all()

        Returns:

        """
        return [obj.to_entity() for obj in query_set]

    def to_dict(self, **kwargs) -> Dict:
        """返回字典对象.

        Returns:

        """
        return self.entity.dict(**kwargs)

