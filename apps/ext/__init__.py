# -*- coding: utf-8 -*-
"""
Copyright (c) XYZ Robotics Inc. - All Rights Reserved
Unauthorized copying of this file, via any medium is strictly prohibited
Proprietary and confidential
Author: Yuhang Wu <yuhang.wu@xyzrobotics.ai>, 2022-08-08

ext目录下的模块是仅用于wcs adaptor的扩展项.
xlhb本身不会使用ext内部的代码.
"""
from .manager.manager import (
    TaskManager as ABCTaskManager,
    OrderManager as ABCOrderManager,
    WorkspaceManager as ABCWorkspaceManager,
)

__all__ = ["ABCTaskManager", "ABCOrderManager", "ABCWorkspaceManager"]
