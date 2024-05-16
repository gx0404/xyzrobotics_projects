#  -*- coding: utf-8 -*-
#  Copyright (c) XYZ Robotics Inc. - All Rights Reserved
#  Unauthorized copying of this file, via any medium is strictly prohibited
#  Proprietary and confidential
#  Author: Yuhang Wu <yuhang.wu@xyzrobotics.ai>, 2022/9/27 下午1:21
import abc


class LazyLoader(abc.ABC):
    """懒加载类

    懒加载类, 用于延迟加载某些类.
    """
    def __getattr__(self, item):
        """获取属性"""
        if "_proxy" not in self.__dict__:
            raise AttributeError("LazyLoader has not been initialized")
        proxy = self.__dict__["_proxy"]
        return getattr(proxy, item)
