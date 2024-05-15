#  -*- coding: utf-8 -*-
#  Copyright (c) XYZ Robotics Inc. - All Rights Reserved
#  Unauthorized copying of this file, via any medium is strictly prohibited
#  Proprietary and confidential
#  Author: Yuhang Wu <yuhang.wu@xyzrobotics.ai>, 2022/9/20 下午6:01
from typing import Dict, Generic, List, Optional, TypeVar

from apps.ext.datastructs.pallet import BasePallet

T = TypeVar("T", bound=BasePallet)


class BaseWorkspace(Generic[T]):
    """工作空间.

    Attributes:
        ws_id (str): 工作空间ID.
        is_ready (bool): 是否就绪, 默认已就绪.
        last_item (Pallet): 最后一个托盘对象.

    Methods:
        ready(): 设置工作空间状态为已就绪.
        not_ready(): 设置工作空间状态为未就绪.
        is_exists(id: str): 判断托盘ID是否已存在.
        add_item(item: T): 添加空间条目.
        get_item(id: str): 获取一个托盘对象, 如果不存在则返回None.
        clear(): 清空当前工作空间中的所有托盘.
        has_pallet(): 判断工作空间是否有托盘.
    """

    def __init__(
        self, ws_id: str, item: Optional[T] = None, is_ready: bool = True
    ) -> None:
        """
        Args:
            ws_id: 工作空间ID.
            item: 托盘对象.
            is_ready: 是否就绪, 默认已就绪.
        """
        self.ws_id: str = ws_id
        self.is_ready: bool = is_ready
        self.__items: List[T] = []
        self.__items_map: Dict[str, T] = {}
        if item:
            self.__items.append(item)
            self.__items_map[item.id] = item

    def ready(self) -> None:
        """设置工作空间状态为已就绪."""
        self.is_ready = True

    def not_ready(self) -> None:
        """设置工作空间状态为未就绪."""
        self.is_ready = False

    @property
    def last_item(self) -> Optional[T]:
        """返回最新的一个托盘.

        Returns:
            Pallet or None: 托盘对象, 如果没有托盘则返回None.
        """
        if len(self.__items) == 0:
            return None
        return self.__items[-1]

    def is_exists(self, id: str) -> bool:
        """判断托盘ID是否已存在

        Args:
            id (str): 托盘ID.

        Returns:
            bool: True, 存在; False, 不存在
        """
        return id in self.__items_map

    def add_item(self, item: T):
        """添加空间条目.

        Args:
            item: 托盘或料箱对象.

        """
        if not self.is_exists(id=item.id):
            self.__items.append(item)
            self.__items_map[item.id] = item

    def get_item(self, id: str) -> Optional[T]:
        """获取一个托盘对象, 如果不存在则返回None.

        Args:
            id (str): 托盘ID.

        Returns:
            Pallet: 托盘对象.
        """
        return self.__items_map.get(id, None)

    def clear(self) -> None:
        """清空当前工作空间中的所有托盘."""
        self.__items = []
        self.__items_map = {}

    def has_pallet(self) -> bool:
        """判断工作空间是否有托盘.

        Returns:
            bool: 有托盘返回True，反之返回False.
        """
        return len(self.__items) != 0
