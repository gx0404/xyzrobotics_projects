#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
    wcs_adaptor.piece_picking.entity
    ~~~~~~~~~~~~~~~~~~~~~

    定义Piece Picking的实体类

    Copyright (c) XYZ Robotics Inc. - All Rights Reserved
    Unauthorized copying of this file, via any medium is strictly prohibited
    Proprietary and confidential
    Developer: Chen Kun <kun.chen@xyzrobotics.ai>, May, 2022
"""
from datetime import datetime
from typing import Any, Optional, Dict

from pydantic import Field

from wcs_adaptor.entity import LiteBaseTask
from wcs_adaptor.enums import TaskStatus, TaskType
from wcs_adaptor.settings import wcs_settings


class PPTask(LiteBaseTask):
    """Piece Picking类

    Attributes:
        task_id(str): 任务编号
        task_index(int): 任务在订单中的下标位置
        order_id(str): (可选)订单ID
        task_type(TaskType): 任务类型
        task_status(TaskStatus): 任务状态
        sku_info(dict): sku信息, 一个或多个组成的列表
        target_num(int): 目标数量
        picking_code(str or None): (可选)抓取空间编码
        picking_position(str): 抓取空间位置
        picking_bin(str): (可选)抓取空间中的区块位置
        placing_code(str or None): (可选)抓取空间编码
        placing_position(str): 放置空间位置
        placing_bin(str): (可选)放置空间的区块位置
        customized_data(dict): (可选)自定义数据字典, 默认为空字典
        create_time(datetime): (可选)创建时间, 默认为当前时间

    Methods:
        report_pick_num: 报告抓取数量
        is_finished: 判断任务是否已完成
        start: 开始任务
        finish: 完成任务
        terminate: 终止任务
        reset: 重置任务状态
        from_ws: 获取抓取料箱空间
        to_ws: 获取放置料箱空间
        switch_ws: 交换抓取空间和放置空间位置
        reset_done_num: 重置任务完成数量
        is_cycle_mode: 判断当前任务是否是循环抓取模式
        to_wcs_json: 返回给wcs需要的任务信息，符合json格式的字典
    """
    task_index: int = Field(description="任务在订单中的排列下标")
    picking_code: Optional[str] = Field(default=None, description="抓取空间编号")
    placing_code: Optional[str] = Field(default=None, description="抓取空间编号")
    picking_position: str = Field(description="抓取空间的位置信息")
    placing_position: str = Field(description="放置空间的位置信息")
    picking_bin: str = Field(default="0", description="抓取空间的格口信息")
    placing_bin: str = Field(default="0", description="放置空间的格口信息")

    def switch_ws(self):
        """交换抓取空间和放置空间"""
        _ = self.from_ws
        object.__setattr__(self, "from_ws", self.to_ws)
        object.__setattr__(self, "to_ws", _)

    def from_ws(self) -> Optional[str]:
        """根据wcs提供的一些信息，如料箱位置、格口位置等信息返回xtf可用的抓取工作空间

        Returns:
            str or None: 返回xtf的抓取工作空间，一般是形如“0”这样的数字型的字符串，如果未找到
                工作空间，则返回空

        """
        # 合成位置+通道的数据格式
        ws = "{}_{}".format(self.picking_position, self.picking_bin)
        # 根据转换表设置为xtf需要的抓取空间
        result = wcs_settings.pp_workspace_conversion.get(ws)
        return result

    def to_ws(self) -> Optional[str]:
        """根据wcs提供的一些信息，如料箱位置、格口位置等信息返回xtf可用的放置工作空间

        Returns:
            str or None: 返回xtf的放置工作空间，一般是形如“0”这样的数字型的字符串，如果未找到
                工作空间，则返回空

        """
        # 合成位置+通道的数据格式
        ws_format = "{}_{}".format(self.placing_position, self.placing_bin)
        # 根据转换表设置为xtf需要的放置空间
        result = wcs_settings.pp_workspace_conversion.get(ws_format)
        return result

    def reset(self):
        """重置任务完成数量"""
        self.done_num = 0
        self.task_status = TaskStatus.STARTED

    def reset_done_num(self) -> None:
        """重置任务完成数量，并重置任务状态为已开始，主要用于循环任务抓取"""
        self.reset()

    def is_cycle_mode(self) -> bool:
        """检查当前是否是循环任务类型（循环任务主要用于压测，和DEMO展示，使物料来回的抓取）

        Returns:
            bool: 返回True或者False, 如果为真，则为循环抓取类型，反之不是

        """
        return self.task_type is TaskType.PP_CYCLE

    def to_wcs_json(self) -> Dict[str, Any]:
        """返回给WCS的JSON数据

        Returns:
            dict: 返回一个字典形式的当前任务信息，注意字典需要符合json的数据规范
        """
        return {
            "task_id": self.task_id,
            "task_type": self.task_type.value,
            "task_type_name": self.task_type.name,
            "task_status": self.task_status.value,
            "task_status_name": self.task_status.name,
            "sku_info": self.sku_info and self.sku_info.dict(),
            "picking_code": self.picking_code,
            "placing_code": self.placing_code,
            "picking_position": self.placing_position,
            "picking_bin": self.placing_bin,
            "target_num": self.target_num,
            "done_num": self.done_num,
            "create_time": self.create_time and str(self.create_time),
            "start_time": self.start_time and str(self.start_time),
            "end_time": self.end_time and str(self.end_time),
            "customized_data": self.customized_data,
        }
