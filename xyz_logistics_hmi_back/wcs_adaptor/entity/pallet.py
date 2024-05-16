#  -*- coding: utf-8 -*-
#  Copyright (c) XYZ Robotics Inc. - All Rights Reserved
#  Unauthorized copying of this file, via any medium is strictly prohibited
#  Proprietary and confidential
#  Author: Yuhang Wu <yuhang.wu@xyzrobotics.ai>, 2022/9/20 下午6:01
from apps.ext.datastructs import BasePallet
from wcs_adaptor.entity.sku import SkuInfo


class Pallet(BasePallet[SkuInfo]):
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
    pass
