#  -*- coding: utf-8 -*-
#  Copyright (c) XYZ Robotics Inc. - All Rights Reserved
#  Unauthorized copying of this file, via any medium is strictly prohibited
#  Proprietary and confidential
#  Author: Yuhang Wu <yuhang.wu@xyzrobotics.ai>, 2022/9/20 下午6:01
import collections
from typing import DefaultDict, Dict, List, Optional, TypeVar, Generic

from apps.ext.datastructs.sku import BaseSKU


T = TypeVar("T", bound=BaseSKU)


class BasePallet(Generic[T]):
    """托盘实体.

    Attributes:
        id(str): 托盘ID.
        pallet_id(str): 托盘ID.
        ws_id(str): 工作空间ID.
        is_clear(bool): 托盘是否已清空.

    Methods:
        clear(): 清空托盘.
        count_from_sku(): 通过sku对象统计的各个sku数量的结果.
        count_from_sku_id(): 通过sku_id统计的各个sku数量的结果.
        count_by_sku_id(sku_id: str): 根据sku_id统计当前托盘上存放的数量.
        count_by_sku(sku: SkuInfo): 根据sku对象统计当前托盘上存放的数量.
        get_sku_by_id(sku_id: str): 根据sku_id获取sku信息.
        add_sku(sku: SkuInfo): 添加sku到当前托盘.
        list_sku(): 获取当前托盘上的所有sku信息.
    """

    def __init__(self, pallet_id: str, ws_id: str, is_clear: bool = True):
        """
        Args:
            pallet_id(str): 托盘ID.
            ws_id(str): 工作空间ID.
            is_clear(bool): 托盘是否已清空.
        """
        self.id: str = pallet_id
        self.pallet_id: str = pallet_id
        self.ws_id: str = ws_id
        self.is_clear: bool = is_clear
        self._sku_counter: Dict[T, int] = collections.Counter()
        self._sku_id_counter: DefaultDict[str, int] = collections.defaultdict(int)

    def clear(self) -> None:
        """清空托盘."""
        self._sku_counter = collections.Counter()
        self._sku_id_counter = collections.defaultdict(int)
        self.is_clear = True

    def count_from_sku(self) -> Dict[T, int]:
        """通过sku对象统计的各个sku数量的结果.

        Examples:
            >>> p = Pallet(pallet_id="1")
            >>> p.add_sku(SkuInfo(sku_id="123", ...))
            >>> p.count_from_sku()
            {
                SkuInfo(sku_id="123",...): 1,
            }
        """
        return dict(self._sku_counter)

    def count_from_sku_id(self) -> Dict[str, int]:
        """通过sku_id统计的各个sku数量的结果.

        Examples:
            >>> p = Pallet(pallet_id="1")
            >>> p.add_sku(SkuInfo(sku_id="123", ...))
            >>> p.count_from_sku_id()
            {
                "123": 1,
            }

        """
        return self._sku_id_counter

    def count_by_sku_id(self, sku_id: str) -> int:
        """根据sku_id统计当前托盘上存放的数量

        Args:
            sku_id: 商品ID

        Returns:

        """
        return self._sku_id_counter[sku_id]

    def count_by_sku(self, sku: T) -> int:
        """根据sku对象统计当前托盘上存放的数量

        Args:
            sku(SkuInfo): sku对象

        Returns:

        """
        return self._sku_counter[sku]

    def get_sku_by_id(self, sku_id: str) -> Optional[T]:
        """根据sku_id获取sku信息

        Args:
            sku_id(str): sku_id

        Returns:
            sku_info(SkuInfo): 该sku对象, 可能为空

        """
        for sku in self._sku_counter:
            if sku_id == sku.sku_id:
                return sku

    def add_sku(self, sku: T):
        """添加sku到当前托盘.

        对应sku数量+1.

        Returns:

        """
        self._sku_counter[sku] += 1

    def list_sku(self) -> List[T]:
        """列出当前托盘上装的所有sku种类

        Returns:

        """
        return list(self._sku_counter.keys())

    def __contains__(self, item: T) -> bool:
        return item in self._sku_counter
