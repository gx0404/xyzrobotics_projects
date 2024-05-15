#  -*- coding: utf-8 -*-
#  Copyright (c) XYZ Robotics Inc. - All Rights Reserved
#  Unauthorized copying of this file, via any medium is strictly prohibited
#  Proprietary and confidential
#  Author: Yuhang Wu <yuhang.wu@xyzrobotics.ai>, 2022/9/20 下午6:01
from apps.ext.datastructs import BaseWorkspace
from wcs_adaptor.entity.pallet import Pallet


class Workspace(BaseWorkspace[Pallet]):
    """工作空间实体.

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
    pass
