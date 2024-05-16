#  -*- coding: utf-8 -*-
#  Copyright (c) XYZ Robotics Inc. - All Rights Reserved
#  Unauthorized copying of this file, via any medium is strictly prohibited
#  Proprietary and confidential
#  Author: Yuhang Wu <yuhang.wu@xyzrobotics.ai>, 2022/9/8 下午3:14
import contextlib
import typing as t


from .protocols import OrderProto, TaskProto, WorkspaceProto
from apps.log import hmi_log


T = t.TypeVar("T", OrderProto, TaskProto, WorkspaceProto, object)


class BaseStore(t.Generic[T]):
    """用于存储任务的存储基类."""
    def top(self) -> T:
        """返回最新的数据."""
        raise NotImplementedError

    def set(self, key: str, value: T):
        """设置值."""
        raise NotImplementedError

    def get(self, key: str, default=None):
        """获取值."""
        raise NotImplementedError

    def remove(self, key: str) -> None:
        """删除值."""
        raise NotImplementedError

    def clear(self) -> None:
        """清空."""
        raise NotImplementedError

    def all(self) -> t.List[T]:
        """返回所有的值."""
        raise NotImplementedError

    def getlist(self) -> t.List[T]:
        """返回所有的值(以列表形式)."""
        raise NotImplementedError

    def getdict(self) -> t.Dict[str, T]:
        """返回所有的值(以字典形式)."""
        raise NotImplementedError

    def __contains__(self, key: str) -> bool:
        """是否包含."""
        raise NotImplementedError

    def __len__(self) -> int:
        """长度."""
        raise NotImplementedError

    def __iter__(self) -> t.Iterator[T]:
        """迭代."""
        raise NotImplementedError

    def __getitem__(self, key: str) -> T:
        """使用 `[]` 获取值."""
        raise NotImplementedError

    def __setitem__(self, key: str, value: T) -> None:
        """使用 `[]` 设置值."""
        raise NotImplementedError

    def __delitem__(self, key: str) -> None:
        """使用 `[]` 删除值."""
        raise NotImplementedError

    def __repr__(self) -> str:
        """返回字符串."""
        return f"{self.__class__.__name__}({self.__dict__})"

    def __str__(self) -> str:
        """返回字符串."""
        return f"{self.__class__.__name__}({self.__dict__})"


class MemoryStore(BaseStore[T]):
    """基于内存的存储"""
    def __init__(self):
        self._data: t.Dict[str, T] = {}

    def top(self) -> t.Optional[T]:
        return next(iter(self._data.values())) if len(self._data) > 0 else None

    def set(self, key: str, value: T) -> None:
        self._data[key] = value
        hmi_log.info(f"set {key} {value}")

    def get(self, key, default=None) -> t.Optional[T]:
        hmi_log.info(f"get {key} {default}")
        return self._data.get(key, default)

    def getlist(self) -> t.List[T]:
        return list(self._data.values())

    def getdict(self) -> t.Dict[str, T]:
        return self._data

    def remove(self, key) -> None:
        with contextlib.suppress(KeyError):
            del self._data[key]

    def clear(self) -> None:
        self._data = {}

    def all(self) -> t.List[T]:
        return self.getlist()

    def __contains__(self, key) -> bool:
        return key in self._data

    def __len__(self) -> int:
        return len(self._data)

    def __iter__(self) -> t.Iterator[T]:
        return iter(self._data.values())

    def __getitem__(self, key: str) -> T:
        return self._data[key]

    def __setitem__(self, key: str, value: T) -> None:
        self._data[key] = value

    def __delitem__(self, key: str) -> None:
        del self._data[key]


# class MongoStore(BaseStore[T]):
#     """基于MongoDB的存储"""
#     def __init__(self, unique_field: str, collection: "Collection"):
#         self.unique_field = unique_field
#         self.collection = collection
#
#     def __get_orig(self):
#
#     def top(self) -> t.Optional[T]:
#         result = self.collection.find_one({})
#
#     def set(self, key: str, value: T):
#         self.collection.update_one({"_id": key}, {"$set": value.dict()}, upsert=True)
#
#     def get(self, key, default=None):
#         return self.collection.find_one({"_id": key})
#
#     def remove(self, key):
#         self.collection.delete_one({"_id": key})
#
#     def clear(self):
#         self.collection.delete_many({})
#
#     def all(self) -> list:
#         return list(self.collection.find({}))


#
# class StoreProxy(BaseStore[T]):
#     """这是一个代理类，对 ``StoreProxy`` 的所有操作都将转发给 ``Store``
#
#     Store 默认返回的数据类型都是 ``Dict``, 基于 ``StoreProxy`` 可以将其转为指定的对象.
#     """
#
#     def __init__(
#         self,
#         store_cls: t.Union[t.Type[MemoryStore], t.Type[MongoStore]],
#
#     ):
#         self.store = store_cls()
#         self._result_cls = None
#
#     def __get_orig(self) -> t.Type[T]:
#         """根据泛型参数获取工作空间类.
#
#         Returns:
#             Type[T]: 类对象.
#         """
#         if self._result_cls is None:
#             self._result_cls = get_args(self.__orig_class__)[0]  # type: ignore
#         return self._result_cls
#
#     # def convetor(self, f: t.Callable):
#     #     @functools.wraps(f)
#     #     def wrapper(*args, **kwargs):
#     #         if self._result_cls is None:
#     #             self._result_cls = get_args(self.__orig_class__)[0]  # type: ignore
#     #         # return self._result_cls
#     #         f: t.Callable
#     #         getattr(self.store, f.name)
#     #         self.store
#     #
#     #     return wrapper
#
#     # @convetor
#     def get(self, key: str, default=None) -> t.Optional[T]:
#         cls = self.__get_orig()
#         if data := self.store.get(key, default):
#             # NOTICE: hardcode
#             #
#             pass
#
#     def set(self, key: str, value: T) -> None:
#         """将 value 转为字典后存储."""
#         value.dict()


def get_store_cls() -> t.Type[MemoryStore]:
    """获取指定的存储类.

    Returns:
        Type[BaseStore]: 存储代理类.
    """
    # TODO: 目前指定获取的是 MemoryStore, 后续需要根据配置获取
    return MemoryStore
