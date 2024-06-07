# -*- coding: utf-8 -*-
"""
Copyright (c) XYZ Robotics Inc. - All Rights Reserved
Unauthorized copying of this file, via any medium is strictly prohibited
Proprietary and confidential
Author: Yuhang Wu <yuhang.wu@xyzrobotics.ai>, 2022-06-16

WCS Adaptor Configuration.
"""
import os
import json
from typing import Optional, Dict

from apps import wcs_log
from apps.settings import WCSBaseSettings, Field, BaseModel
from wcs_adaptor.exceptions import WCSError


class TaskManagerConfig(BaseModel):
    # 默认为1, 如果设置为0, 则容量大小不作限制.
    size: int = Field(default=1, description="TaskManager内部允许存储的最大任务数.", ge=0)

    def __repr__(self) -> str:
        return self.json(indent="  ")


class OfflinePlanningConfig(BaseModel):
    max_pallet_num: int = Field(default=10, ge=0, title="最大托盘数量")
    conversion_dict: dict = Field(default={}, description="工作空间转换")


class WCSSettings(WCSBaseSettings):
    """集中管理WCS Adaptor配置信息.

    Examples:
        >>> from wcs_adaptor import wcs_settings
        >>> wcs_settings.task_manager_config
        {
          "size": 1
        }
    """
    task_manager_config: TaskManagerConfig = Field(
        default=TaskManagerConfig(),
        description="用于加载TaskManager的配置"
    )
    # 用于离线混码规划的配置
    offline_order_planning_config: OfflinePlanningConfig = Field(
        default=OfflinePlanningConfig(),
        description="用于加载离线订单规划的配置"
    )
    # 分拣站工作空间转换表，将与用户约定的坐标格式转换为xtf可识别的工作空间
    # 下面键的组成方式是【位置_通道】，值的组成方式是xtf抓取的【工作空间】
    # 【位置】信息的来源在picking_position和placing_position这个字段
    # 【通道】信息的来源在picking_bin和placing_bin这个字段
    # 注：picking_position字段可能会根据项目被重命名
    #
    # 场景：抓取位置是一个来料箱，包含10个格口；放置位置是输送线，没有格口
    #      如果需要表示从抓取位置抓取来料箱的3号格口到输送线上，抓取空间就
    #      可以是【'0_3': '0'】，放置空间就可以是【'1_0': '10'】
    #
    # 场景：抓取位置和放置位置都是一个来料箱，且没有格口，那么抓取空间就可以
    #      是【'0_0': '0'】，放置空间就可以是【'1_0': '1'】
    #
    # 场景：抓取位置是一个货架，包含10个格口，还有正反面之分；放置位置是一个
    #      输送线首端；如果需要从货架的反面抓取10号格口的货物，那么抓取空间
    #      就可以是【'0_19': '19'】，如果需要向输送线首端放置一个货物，那
    #      么放置空间就可以是【'1_0': '20'】
    pp_workspace_conversion: Dict[str, str] = {
        '0_0': '0',
        '1_0': '1',
        '2_0': '2',
        '3_0': '3'
    }
    # 分拣站与XTF相关接口，设置接口启用状态
    pp_xtf_api_map: Dict[str, bool] = {
        "get_task_info": True,
        "allow_pick": True,
        "allow_move": True,
        "allow_release": True,
        "report_step_outcome": True,
        "error_handle": True,
        "error": True,
    }


wcs_settings: Optional[WCSSettings] = None


def init_settings() -> WCSSettings:
    global wcs_settings

    file_path = WCSSettings.Config.config_file_path
    config_path = WCSSettings.Config.config_path
    # 不存在则生成一个空配置文件.
    if not os.path.exists(file_path):
        os.makedirs(config_path, exist_ok=True)
        with open(file_path, "w", encoding="utf-8") as f:
            f.write("{}")
        wcs_log.warning(
            "Could not found wcs_adaptor.json file,"
            " has been generated empty configuration file."
        )
    if wcs_settings is None:
        try:
            wcs_settings = WCSSettings()
        except json.JSONDecodeError:
            raise WCSError(
                f"WCS Adaptor配置文件加载失败, 请检查配置文件是否完整"
                f" {WCSSettings.Config.config_file_path}"
            )
        wcs_settings.dumps()
    return wcs_settings
