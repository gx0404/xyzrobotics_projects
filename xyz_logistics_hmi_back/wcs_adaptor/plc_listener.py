# -*- coding: utf-8 -*-
import time
from functools import partial
from threading import Thread

from apps import cached_app
from apps.globals import mp
from apps.log import plc_log, wcs_log
from apps.settings import settings
from apps.utils import node, plc
from apps.utils.util import get_robot_status
from apps.base_app.views.command.command_api import stop_xtf
from wcs_adaptor.zh_msg import ALL_ERROR

device_id = settings.PLC_DEVICE_ID


def plc_monitor_robot_singal(interval: int = 0.5):
    """通过PLC监听机器人信号, 由于后端启动后PLC、Robot等相关节点还不会立即启动，
    所以需要先监听其状态，只有在状态已就绪的情况下才能正常读取信号，否则不读取信号

    Args:
        interval(int): 轮询间隔，默认是0.5秒一次
    """
    robot_status = None
    app = cached_app()
    while 1:
        with app.app_context():
            current_robot_status = get_robot_status()
            if robot_status != current_robot_status:
                robot_status = current_robot_status
                mp.robot_status(robot_status)
            time.sleep(interval)


def plc_monitor_scram_signal(interval: int = 0.5):
    """get_plc实现获取机器人急停信号 。一旦获取到机器人急停，抛出异常给HMI和WCS
    系统急停信号与机器人急停信号

    Args:
        interval(int): 轮询间隔，默认是0.5秒一次

    """
    app = cached_app()
    while True:
        with app.app_context():
            if app.status == "ready":
                #import ipdb;ipdb.set_trace()
                try:
                    # Read air signal
                    air_signal = plc.read_signal(65010)
                    # Stop xtf and raise exception if scram
                    real_low_air = False
                    if not air_signal:
                        current_time = time.time()
                        while True:
                            air_signal = plc.read_signal(65010)
                            if time.time()-current_time> 3:
                                real_low_air = True
                                break
                            if not air_signal:
                                continue
                            else:
                                real_low_air = False
                                break    
                        if not real_low_air:
                            plc_log.warn("气压信号不稳定")
                            continue    
                        plc_log.warn("捕捉到气压低信号")
                        # Stop xtf
                        stop_xtf()
                        node.stop_robot_node(force_stop=True)
                        error_info = ALL_ERROR.get("70010")
                        error_data = {
                            "code": "70010",
                            "msg_type": "error",
                            "error_handle_method": 0,
                            "en_msg": error_info["error_msg"],
                            "zh_msg": error_info["msg"],
                            "zh_tip": error_info["tip"],
                            "ja_msg": error_info["msg"],
                            "ja_tip": error_info["msg"],
                            "class": error_info.get("class", "motion"),
                        }
                        mp.system.error(error_data)
                except:
                    plc_log.error(
                        "Fail to read signal {}".format(
                            settings.PLC_ADDRESS_DICT["estop"]
                        )
                    )
                    stop_xtf()
                    node.stop_robot_node(force_stop=True)
                    error_info = ALL_ERROR.get("70020")
                    error_data = {
                        "code": "70020",
                        "msg_type": "error",
                        "error_handle_method": 0,
                        "en_msg": error_info["error_msg"],
                        "zh_msg": error_info["msg"],
                        "zh_tip": error_info["tip"],
                        "ja_msg": error_info["msg"],
                        "ja_tip": error_info["msg"],
                        "class": error_info.get("class", "motion"),
                    }
                    mp.system.error(error_data)
                    mp.order.error(f"气压检测异常")
            # 注意: 推送三色灯不需要判断系统状态
            try:
                # Read plc light and sync to HMI
                plc_light = [
                    plc.read_signal(settings.PLC_ADDRESS_DICT["green"]),
                    plc.read_signal(settings.PLC_ADDRESS_DICT["yellow"]),
                    plc.read_signal(settings.PLC_ADDRESS_DICT["red"]),
                ]
                if plc_light != app.plc_light:
                    mp.rgb_light(*plc_light)
                    app.plc_light = plc_light
            except Exception as e:
                plc_log.error("Fail to update plc light, {}".format(repr(e)))

        time.sleep(interval)


def plc_set_rgblight(light):
    """set_plc实现三色灯亮不同颜色的灯
    直接用plc_set_redlight()即可开启红灯

    Args:
        light: "red", "yellow", "green"

    """

    if light == "red":
        if settings.PLC_THREAD_ENABLE:
            plc.set_signal(settings.PLC_ADDRESS_DICT["red"], 1)
            plc.set_signal(settings.PLC_ADDRESS_DICT["yellow"], 0)
            plc.set_signal(settings.PLC_ADDRESS_DICT["green"], 0)
            plc.set_signal(settings.PLC_ADDRESS_DICT["buzzer"], 1)
    elif light == "yellow":
        if settings.PLC_THREAD_ENABLE:
            plc.set_signal(settings.PLC_ADDRESS_DICT["red"], 0)
            plc.set_signal(settings.PLC_ADDRESS_DICT["yellow"], 1)
            plc.set_signal(settings.PLC_ADDRESS_DICT["green"], 0)
            plc.set_signal(settings.PLC_ADDRESS_DICT["buzzer"], 0)
    elif light == "green":
        if settings.PLC_THREAD_ENABLE:
            plc.set_signal(settings.PLC_ADDRESS_DICT["red"], 0)
            plc.set_signal(settings.PLC_ADDRESS_DICT["yellow"], 0)
            plc.set_signal(settings.PLC_ADDRESS_DICT["green"], 1)
            plc.set_signal(settings.PLC_ADDRESS_DICT["buzzer"], 0)
    else:
        plc_log.error("{} is not defined".format(light))
        return
    plc_log.info("Set {}".format(light))


# 以下方式创建了plc_set_redlight等方法，使用时直接plc_set_redlight()即可
plc_set_redlight = partial(plc_set_rgblight, light="red")
plc_set_yellowlight = partial(plc_set_rgblight, light="yellow")
plc_set_greenlight = partial(plc_set_rgblight, light="green")

get_plc = read_plc = get_signal = plc.read_signal
set_plc = write_plc = write_signal = plc.set_signal


def start_plc_listener_thread(interval: int = 0.5):
    """开启PLC监听线程

    Args:
        interval(int): 设置线程中PLC信号的轮询间隔，默认是0.5秒一次
    """
    wcs_log.info("Starting PLC listener thread.")
    t = Thread(
        target=plc_monitor_scram_signal,
        args=(interval,),
        daemon=True,
        name="PlcMonitor",
    )
    t.start()
    
    t_1 = Thread(
        target=heartbeat_plc,
        args=(),
        daemon=True,
        name="heartbeat",
    )
    t_1.start()

#plc心跳信号
def heartbeat_plc():
    while True:
        time.sleep(1)
        try:
            plc.set_signal(65030, 1)
            time.sleep(1)
            plc.set_signal(65030, 0)
            time.sleep(1)
        except:
            plc_log.error("set plc heart io fault")       


def start_robot_status_thread(interval: int = 0.5):
    """开启监听机械臂状态的线程，本质也是使用PLC轮询监听机械臂相关的地址信号，监听到信号变动后，
    会主动广播，即推送websocket信号给HMI上

    Args:
        interval(int): 设置线程中PLC信号的轮询间隔，默认是0.5秒一次
    """
    wcs_log.info("Starting robot status listener thread")
    t = Thread(
        target=plc_monitor_robot_singal,
        args=(interval,),
        daemon=True,
        name="RobotMonitor",
    )
    t.start()
    