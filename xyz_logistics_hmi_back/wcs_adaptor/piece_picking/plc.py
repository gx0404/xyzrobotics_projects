#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
    wcs_adaptor.piece_picking.plc
    ~~~~~~~~~~~~~~~~~~~~~

    包含PLC读写方法

    Copyright (c) XYZ Robotics Inc. - All Rights Reserved
    Unauthorized copying of this file, via any medium is strictly prohibited
    Proprietary and confidential
    Developer: Chen Kun <kun.chen@xyzrobotics.ai>, June, 2022
"""
import time
import importlib
from functools import wraps

import rospy

from apps.log import plc_log

logger = plc_log
PLC_READ_ERROR = None    # 记录PLC是否有读取错误
PLC_WRITE_ERROR = None   # 记录PLC是否有写入错误
PLC_SHORT_LOG = True     # 是否启用短日志提示 

def catch_read_error(func):
    @wraps(func)
    def _wrapper(*args, **kwargs):
        global PLC_READ_ERROR, PLC_SHORT_LOG
        result = None
        error_message = None
        try:
            result = func(*args, **kwargs)
        except ImportError as e:
            error_message = "读取PLC地址信号失败，与PLC的通信模块导入失败"
            if PLC_READ_ERROR != 1:
                PLC_READ_ERROR = 1
                logger.error(error_message, exc_info=True)
        except rospy.service.ServiceException as e:
            error_message = "读取PLC地址信号失败，与PLC通信模块未正常开启"
            if PLC_READ_ERROR != 2:
                PLC_READ_ERROR = 2
                logger.error(error_message, exc_info=True)
        except Exception as e:
            error_message = "写入PLC地址信号失败，出现未知异常"
            if PLC_READ_ERROR != repr(e):
                PLC_READ_ERROR = repr(e)
                logger.error(error_message, exc_info=True)
        else:
            if PLC_READ_ERROR:
                PLC_READ_ERROR = None
        return result, error_message
    return _wrapper

def catch_write_error(func):
    @wraps(func)
    def _wrapper(*args, **kwargs):
        global PLC_WRITE_ERROR, PLC_SHORT_LOG
        try:
            func(*args, **kwargs)
        except ImportError as e:
            error_message = "写入PLC地址信号失败，连接PLC的IO服务模块导入失败"
            if PLC_WRITE_ERROR != 1:
                PLC_WRITE_ERROR = 1
                logger.error(error_message, exc_info=True)
        except rospy.service.ServiceException as e:
            error_message = "写入PLC地址信号失败，PLC的IO服务模块未正常开启"
            if PLC_WRITE_ERROR != 2:
                PLC_WRITE_ERROR = 2
                logger.error(error_message, exc_info=True)
        except Exception as e:
            error_message = "写入PLC地址信号失败，出现未知异常"
            if PLC_WRITE_ERROR != repr(e):
                PLC_WRITE_ERROR = repr(e)
                logger.error(error_message, exc_info=True)
        else:
            error_message = None
            if PLC_WRITE_ERROR:
                PLC_WRITE_ERROR = None
        return error_message
    return _wrapper


@catch_read_error
def read_plc(address, length=1, device_id="2"):
    """读取PLC地址信号

    Args:
        address(int): 读取的PLC起始地址，数值范围为0-65535
        length(int): 读取地址长度
        device_id(str): (可选)默认使用“2”，即使用PLC通信方式

    Returns:
        value(tuple or None): 返回读取的信号值，如果读取失败则返回None 

    """
    io_client = importlib.import_module("xyz_io_client.io_client")
    if length > 1:
        get_digit_inputs = getattr(io_client, "get_digit_inputs")
        value = get_digit_inputs(
            device_id=device_id,
            start_port=address,
            length=length
        )
    else:
        get_digit_input = getattr(io_client, "get_digit_input")
        value = get_digit_input(device_id=device_id, start_port=address)
    return value


@catch_write_error
def write_plc(address, value, device_id="2"):
    """向PLC地址写入信号

    Args:
        address(int): 读取的PLC起始地址，数值范围为0-65535
        value(int or list): 期望写入的数值，如果数值是整数，则只写入起始地址，如果
            数值是一个列表或者元组，则写入从起始地址开始写入
        device_id(str): (可选)默认使用“2”，即使用PLC通信方式
    
    Raise:
        error_message(str or None): 如果出现异常，则返回异常信息

    """
    io_client = importlib.import_module("xyz_io_client.io_client")
    if isinstance(value, list):
        set_digit_outputs = getattr(io_client, "set_digit_outputs")
        set_digit_outputs(
            device_id=device_id,
            start_port=address,
            digit_values=value,
            length=len(value)
        )
    else:
        set_digit_output = getattr(io_client, "set_digit_output")
        set_digit_output(
            device_id=device_id,
            start_port=address,
            digit_value=value
        )


class PlcClient(object):
    """PLC通信模块，封装了读取、写入数据的方法，以及循环读取
    """

    def read(self, address, length=1, device_id="2", **kwargs):
        value, error_message = read_plc(address, length, device_id)
        return value, error_message

    def write(self, address, value, device_id="2", **kwargs):
        error_message = write_plc(address, value, device_id)
        return error_message

    def cycle_read(self, address, except_value, wait_time=0, interval=0.1):
        result = False
        start_time = end_time = int(time.time())
        while wait_time == 0 or wait_time > end_time - start_time:
            value = self.read(address)
            if value == except_value:
                result = True
                break
            end_time = int(time.time())
            time.sleep(interval)
        return result