#  -*- coding: utf-8 -*-
#  Copyright (c) XYZ Robotics Inc. - All Rights Reserved
#  Unauthorized copying of this file, via any medium is strictly prohibited
#  Proprietary and confidential
#  Author: Yuhang Wu <yuhang.wu@xyzrobotics.ai>, 2022/9/1 上午9:41
import copy
import warnings
from typing import Type, Any

import pydantic
from pydantic import BaseModel
from pydantic.main import ModelMetaclass


class ProxyMetaclass(type):
    def __new__(mcs, name, bases, attrs):
        # sourcery skip: class-method-first-arg-name, inline-immediately-returned-variable
        pydantic_bases = []
        for base in reversed(bases):
            # if "Proxy" in str(base):
            if issubclass(base, ProxyMetaclass) or issubclass(base, Proxy):
                hasattr(base, "_original_class") and pydantic_bases.append(
                    base._original_class
                )

        if pydantic_bases:
            warnings.filterwarnings("ignore", category=RuntimeWarning)
            # 动态创建 pydantic.BaseModel 类
            pydantic_class = pydantic.create_model(
                name,
                __base__=tuple(pydantic_bases),
                # __config__=pydantic_bases[0].__config__,
                __inner_flag__=1,
                **attrs
            )
            assert issubclass(pydantic_class, BaseModel)
            warnings.filterwarnings("always", category=RuntimeWarning)

            return readonly_property(pydantic_class)
        return super().__new__(mcs, name, bases, attrs)

    def __getitem__(self, item):
        # 构建一个新的类
        self.__orig_class__ = item
        return self


class Proxy(metaclass=ProxyMetaclass):
    __obj: BaseModel

    def __setattr__(self, key, value):
        if key in self.__obj.__fields__:
            raise AttributeError(
                f"'{key}' is not allowed to be assigned, use function call instead."
            )
        super().__setattr__(key, value)

    def __getattr__(self, name):
        if "obj" in name:
            return super(Proxy, self).__getattr__(name)
        assert isinstance(self.__obj, BaseModel), "`__obj` must be a pydantic.BaseModel"
        value = getattr(self.__obj, name)
        if callable(value):
            method = getattr(self.__obj, "__getattr__")(name)
            if method is None:
                return value
        # 如果是可变数据类型，返回该值的浅拷贝副本
        return copy.copy(value) if isinstance(value, (dict, list, set)) else value

    def _get_current_object(self):
        """获取被代理对象"""
        return self.__obj

    def _get_original_class(self):
        """获取被代理对象的原始类"""
        return self.__obj.__class__

    @classmethod
    def __get_validators__(cls) -> 'CallableGenerator':
        yield cls.validate

    @classmethod
    def validate(cls: Type['Model'], value: Any) -> 'Model':
        return cls(**value)

    def __str__(self):
        return str(self.__obj)

    def __repr__(self):
        return repr(self.__obj)

    __delattr__ = lambda self, name: delattr(self.__obj, name)
    # __str__ = __repr__ = lambda self: self.__obj.__str__()
    __lt__ = lambda self, other: self.__obj.__lt__(other)
    __le__ = lambda self, other: self.__obj.__le__(other)
    __eq__ = lambda self, other: self.__obj.__eq__(other)
    __ne__ = lambda self, other: self.__obj.__ne__(other)
    __gt__ = lambda self, other: self.__obj.__gt__(other)
    __ge__ = lambda self, other: self.__obj.__ge__(other)
    __cmp__ = lambda x, o: cmp(x._get_current_object(), o)  # noqa
    __hash__ = lambda x: hash(x._get_current_object())
    __call__ = lambda x, *a, **kw: x._get_current_object()(*a, **kw)
    # __len__ = lambda x: len(x._get_current_object())
    __getitem__ = lambda x, i: x._get_current_object()[i]
    __iter__ = lambda x: iter(x._get_current_object())
    __contains__ = lambda x, i: i in x._get_current_object()
    __add__ = lambda x, o: x._get_current_object() + o
    __sub__ = lambda x, o: x._get_current_object() - o
    __mul__ = lambda x, o: x._get_current_object() * o
    __floordiv__ = lambda x, o: x._get_current_object() // o
    __mod__ = lambda x, o: x._get_current_object() % o
    __divmod__ = lambda x, o: x._get_current_object().__divmod__(o)
    __pow__ = lambda x, o: x._get_current_object() ** o
    __lshift__ = lambda x, o: x._get_current_object() << o
    __rshift__ = lambda x, o: x._get_current_object() >> o
    __and__ = lambda x, o: x._get_current_object() & o
    __xor__ = lambda x, o: x._get_current_object() ^ o
    __or__ = lambda x, o: x._get_current_object() | o
    __div__ = lambda x, o: x._get_current_object().__div__(o)
    __truediv__ = lambda x, o: x._get_current_object().__truediv__(o)
    __neg__ = lambda x: -(x._get_current_object())
    __pos__ = lambda x: +(x._get_current_object())
    __abs__ = lambda x: abs(x._get_current_object())
    __invert__ = lambda x: ~(x._get_current_object())
    __complex__ = lambda x: complex(x._get_current_object())
    __int__ = lambda x: int(x._get_current_object())
    __long__ = lambda x: long(x._get_current_object())  # noqa
    __float__ = lambda x: float(x._get_current_object())
    __oct__ = lambda x: oct(x._get_current_object())
    __hex__ = lambda x: hex(x._get_current_object())
    __index__ = lambda x: x._get_current_object().__index__()
    __coerce__ = lambda x, o: x._get_current_object().__coerce__(x, o)
    __enter__ = lambda x: x._get_current_object().__enter__()
    __exit__ = lambda x, *a, **kw: x._get_current_object().__exit__(*a, **kw)
    __radd__ = lambda x, o: o + x._get_current_object()
    __rsub__ = lambda x, o: o - x._get_current_object()
    __rmul__ = lambda x, o: o * x._get_current_object()
    __rdiv__ = lambda x, o: o / x._get_current_object()
    __rtruediv__ = __rdiv__
    __rfloordiv__ = lambda x, o: o // x._get_current_object()
    __rmod__ = lambda x, o: o % x._get_current_object()
    __rdivmod__ = lambda x, o: x._get_current_object().__rdivmod__(o)
    __copy__ = lambda x: copy.copy(x._get_current_object())
    __deepcopy__ = lambda x, memo: copy.deepcopy(x._get_current_object(), memo)
    __nonzero__ = lambda x: bool(x._get_current_object())


def readonly_property(pydantic_class):
    """被装饰的 :class:`BaseModel` 字段只读.

    只读字段可以通过 `obj.field` 访问，但不能通过 `obj.field = value` 设置值。
    如果需要修改只读字段，请使用函数调用的方式进行修改。

    Examples:
        >>> @readonly_property
        ... class MyModel(BaseModel):
        ...     a: int = 1
        ...     def set_a(self, value):
        ...         self.a = value
        >>> obj = MyModel()
        >>> obj.a = 2
        Traceback (most recent call last):
        ...
        AttributeError: 'a' is not allowed to be assigned, use function call instead.
        >>> obj.set_a(2)
        >>> print(obj.a)
        2
    """
    # if issubclass(pydantic_class, Proxy):
    #     # 获取原始类
    #     # 重新继承至原始类
    #     pydantic_class = type(
    #         pydantic_class.__name__,
    #         (pydantic_class._original_class, ),
    #         dict(pydantic_class.__dict__)
    #     )

    def __init__(self, *args, **kwargs):
        object.__setattr__(self, "_Proxy__obj", pydantic_class(*args, **kwargs))

    proxy_class = type(
        pydantic_class.__name__ + "Proxy",
        (Proxy,),
        {"__init__": __init__, "_original_class": pydantic_class}
    )
    return proxy_class


class ReadonlyModelMetaclass(ModelMetaclass):
    """被装饰的 :class:`BaseModel` 字段只读.

    Examples:
        >>> class Task(BaseModel, metaclass=ReadonlyModelMetaclass):
        ...     a: int = 1
        ...     def set_a(self, value):
        ...         self.a = value
        >>> task = Task()
        >>> task.a = 2
        Traceback (most recent call last):
        ...
        AttributeError: 'a' is not allowed to be assigned, use function call instead.
        >>> task.set_a(2)
        >>> print(task.a)
        2
        >>> class SubTask(Task):
        ...     b: int = 2
        ...     def set_b(self, value):
        ...         self.b = value
        >>> subtask = SubTask()
        >>> subtask.b = 3
        Traceback (most recent call last):
        ...
        AttributeError: 'b' is not allowed to be assigned, use function call instead.
        >>> subtask.a = 4
        Traceback (most recent call last):
        ...
        AttributeError: 'a' is not allowed to be assigned, use function call instead.
        >>> subtask.set_a(4)
        >>> subtask.set_b(3)
        >>> print(subtask)
        a=4 b=3
    """

    def __new__(mcs, name, bases, namespace, **kwargs):
        pd_cls = super().__new__(mcs, name, bases, namespace, **kwargs)
        if "__inner_flag__" in namespace:
            return pd_cls
        else:
            return readonly_property(pd_cls)


if __name__ == "__main__":

    # @readonly_property
    class Task(BaseModel, metaclass=ReadonlyModelMetaclass):
        status: str = "pending"

        def set_status(self, value):
            self.status = value

    # task = Task()
    # task.status = "done"  # AttributeError: 'status' is not allowed to be assigned, use function call instead.

    from datetime import datetime
    from pydantic import Field
    import inspect


    class SubTask(Task):
        created_at: datetime = Field(default_factory=datetime.now)

    subtask = SubTask()
    # TODO: SubTask._original_class 错误, 是 BaseModel 类，而不是 Proxy 类
    print(inspect.getmro(SubTask))

    subtask.set_status("done")
    print(subtask)
    print(subtask.status)
    # subtask.created_at = datetime.now()

    class SubSubTask(SubTask):
        updated_at: datetime = Field(default_factory=datetime.now)
        a: int

        def set_status(self, value):
            self.status = value

        def set_updated_at(self, value):
            self.updated_at = value

        def set_a(self, value):
            self.a = value

    subsubtask = SubSubTask(a=1)
    print(subsubtask)
    subsubtask.set_status("undone")
    subsubtask.set_updated_at("2020-01-01")
    subsubtask.set_a("10")
    print(subsubtask)
