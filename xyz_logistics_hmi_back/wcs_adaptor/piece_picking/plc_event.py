#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
    wcs_adaptor.piece_picking.plc
    ~~~~~~~~~~~~~~~~~~~~~

    包含对PLC事件的一些操作

    Copyright (c) XYZ Robotics Inc. - All Rights Reserved
    Unauthorized copying of this file, via any medium is strictly prohibited
    Proprietary and confidential
    Developer: Chen Kun <kun.chen@xyzrobotics.ai>, May, 2022
"""

from wcs_adaptor.piece_picking.plc import PlcClient

class PlcEvent(PlcClient):
    """提供PLC相关事件，比如项目中需要自定义的一些PLC处理事件，可以在该类中实现相关事件
    """
    pass