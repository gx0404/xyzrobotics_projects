#  -*- coding: utf-8 -*-
#  Copyright (c) XYZ Robotics Inc. - All Rights Reserved
#  Unauthorized copying of this file, via any medium is strictly prohibited
#  Proprietary and confidential
#  Author: Yuhang Wu <yuhang.wu@xyzrobotics.ai>, 2022/11/4 上午8:44
from typing import Optional

from xyz_io_client.io_client import (
    get_digit_input,
    get_digit_inputs,
    set_digit_output,
    set_digit_outputs,
)
from apps.settings import settings


def read_signal(address, length=1):
    """读取指定地址PLC信号数值"""

    if length > 1:
        res = get_digit_inputs(settings.PLC_DEVICE_ID, address, length)
        return res[0]
    else:
        value = get_digit_input(device_id=settings.PLC_DEVICE_ID, start_port=address)
        return value


def set_signal(address, value=1, length=1):
    """写入指定地址PLC信号数值"""
    if length > 1:
        set_digit_outputs(settings.PLC_DEVICE_ID, address, length, [value])
    else:
        set_digit_output(
            device_id=settings.PLC_DEVICE_ID, start_port=address, digit_value=value
        )


def close_buzzer(device_id: Optional[str] = None, value: int = 0):
    """关闭蜂鸣器"""
    if not device_id:
        device_id = settings.PLC_DEVICE_ID
    set_digit_output(
        device_id=device_id,
        start_port=settings.PLC_ADDRESS_DICT["buzzer"],
        digit_value=value,
    )
    set_digit_output(
        device_id=device_id,
        start_port=settings.PLC_ADDRESS_DICT["red"],
        digit_value=value,
    )


def disable_grating(device_id: Optional[str] = None, address = None):
    """禁用光栅"""
    if not device_id:
        device_id = settings.PLC_DEVICE_ID
    if not address:
        address = settings.PLC_ADDRESS_DICT["grating"]
    set_digit_output(device_id=device_id, start_port=address, digit_value=0)


def enable_grating(device_id: Optional[str] = None, address = None):
    """开启光栅"""
    if not device_id:
        device_id = settings.PLC_DEVICE_ID
    if not address:
        address = settings.PLC_ADDRESS_DICT["grating"]
    set_digit_output(device_id=device_id, start_port=address, digit_value=1)
