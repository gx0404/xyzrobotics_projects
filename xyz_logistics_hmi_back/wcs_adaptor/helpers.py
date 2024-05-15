# -*- coding: utf-8 -*-
"""
Copyright (c) XYZ Robotics Inc. - All Rights Reserved
Unauthorized copying of this file, via any medium is strictly prohibited
Proprietary and confidential
Author: Yuhang Wu <yuhang.wu@xyzrobotics.ai>, 2022-05-09
"""
from typing import Callable

from apps.ext.manager.helpers import ManagerHelper as _ManagerHelper
from apps.log import wcs_log


class ManagerHelper(_ManagerHelper):
    """ManagerHelper"""
    @staticmethod
    def backup_manager():
        return _ManagerHelper.backup_manager()

    @staticmethod
    def backup_manager_wrapper() -> Callable:
        return _ManagerHelper.backup_manager_wrapper()

    @staticmethod
    def load_task():
        return _ManagerHelper.load_task()


# NOTICE: 以下方法仅支持基于内存的 manager 使用.
# 备份 manager
backup_manager = ManagerHelper.backup_manager
# 用于备份 manager 的装饰器
backup_manager_wrapper = ManagerHelper.backup_manager_wrapper
# 用于从数据库加载任务
load_task = ManagerHelper.load_task


def calculate_sku_order(sku_info):
    """
    Args:
        sku_info(list): [
            {
                "width": float
                "length": float
                "height": float
                "weight": float
                "sku_id": str
                "sku_num": int
            },
            {

            }
        ]
    Return:
        data(list):
            [
                {
                    "sku_id": "6904579670800",
                    "serial_num": 0,
                    "pallet_index": 0
                },
                {
                    "sku_id": "6907179110814",
                    "serial_num": 1,
                    "pallet_index": 0
                },
                {
                    "sku_id": "6907179110814",
                    "serial_num": 2,
                    "pallet_index": 1
                },
                {
                    "sku_id": "6907179110814",
                    "serial_num": 3,
                    "pallet_index": 1
                }
            ]
        pallet_num: int
    """
    print(sku_info)
    import math

    # Define pallet length, width, height
    pallet_length = 1
    pallet_width = 1
    pallet_height = 1
    pallet_volume = pallet_length * pallet_width * pallet_height
    # sort sku
    # 离线混码排序规则如下
    # L, W, H相同的连续来
    # L*W大的优先来，L*W相等的情况下，高度相同的一起来（有低到高）
    import copy
    sku_info_list = copy.deepcopy(sku_info)
    total_volume = 0  # m**3
    for sku in sku_info_list:
        sku["area"] = sku["width"] / 1000.0 * sku["length"] / 1000.0
        sku["volume"] = sku["area"] * sku["height"] / 1000.0
        total_volume += sku["volume"] * sku["sku_num"]

    # 托盘数量估计做保守估计： round（所有箱子体积/托盘容积） + 1
    pallet_num = math.ceil(total_volume / pallet_volume) + 1
    wcs_log.info(
        "Total sku volumn is {}. Each pallet volume is {}. We Need {} pallets".format(
            total_volume,
            pallet_volume,
            pallet_num
        )
    )

    # L*W大的优先来
    sku_sort_by_area = sorted(
        sku_info_list,
        key=lambda x: x["area"],
        reverse=True
    )
    # L*W相等的情况下，高度相同的一起来（有低到高）
    tmp = []
    sku_sort_by_height = []
    area_index = -1
    recent_area = 0
    for idx, sku in enumerate(sku_sort_by_area):
        if recent_area != sku["area"]:
            # Sort same area sku by height
            if area_index != -1:
                sku_sort_by_height[area_index] = sorted(
                    sku_sort_by_height[area_index],
                    key=lambda x: x["height"]
                )
            # Update index and recent area
            recent_area = sku["area"]
            area_index += 1
            sku_sort_by_height.append([])
            sku_sort_by_height[area_index].append(sku)
        else:
            sku_sort_by_height[area_index].append(sku)
        if idx == len(sku_sort_by_area) - 1:  # Last one:
            sku_sort_by_height[area_index] = sorted(
                sku_sort_by_height[area_index],
                key=lambda x: x["height"]
            )
    data = []
    serial_num = 0
    pallet_index = 0
    area_sum = 0
    for sku_with_same_area in sku_sort_by_height:
        for sku in sku_with_same_area:
            for i in range(sku["sku_num"]):
                if area_sum + sku["volume"] > pallet_volume:
                    pallet_index += 1
                    area_sum = sku["volume"]
                else:
                    area_sum += sku["volume"]
                organized_sku_info = {
                    "sku_id": sku["sku_id"],
                    "serial_num": serial_num,
                    "pallet_index": pallet_index
                }
                data.append(organized_sku_info)
                serial_num += 1
    return data, pallet_num
