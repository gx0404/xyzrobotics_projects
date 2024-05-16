import os
import signal
import datetime
import subprocess
import warnings
from distutils.dir_util import copy_tree
from multiprocessing import Pool
from typing import Optional, TYPE_CHECKING

import roslaunch
import rospy

import apps
from apps.app import cached_app
from apps.enums import RobotStatus
from apps.exceptions import XYZConfigError
from apps.log import outside_log, hmi_log
from apps.settings import settings

if TYPE_CHECKING:
    from apps.app import Application
    from apps.settings import Settings


def get_robot_status(device_id: str = "2") -> RobotStatus:
    """获取机器人状态，涉及的状态如下

        "STOPPED": "停止",
        "NORMAL": "正常",
        "MANUAL": "手动",
        "DISABLED": "非使能",
        "NOT_CYCLE": "非循环",
        "ERROR": "报错",
        "PAUSED": "暂停",
        "ESTOPPED": "急停",
        "COLLISION": "碰撞",
        "UNKNOWN": "未知"
        "": "未知"

    Args:
        device_id(str): (可选)设备号，这个是由于xyz_io模块封装的原因，默认传2，指使用PLC通信


    """
    try:
        from xyz_io_client.io_client import (
            is_auto_mode,
            is_enabled,
            is_in_cycle,
            is_in_pause,
            is_estop,
            is_error,
            is_collision,
            is_normal,
        )
    except ImportError:
        return RobotStatus.IMPORT_IO_ERROR

    try:
        # 检查机器人是否处理异常状态，如果不是，那么返回NORMAL
        if not is_auto_mode(device_id):
            robot_status = RobotStatus.MANUAL
        elif not is_enabled(device_id):
            robot_status = RobotStatus.DISABLED
        elif not is_in_cycle(device_id):
            robot_status = RobotStatus.NOT_CYCLE
        elif is_error(device_id):
            robot_status = RobotStatus.ERROR
        elif is_in_pause(device_id):
            robot_status = RobotStatus.PAUSED
        elif is_estop(device_id):
            robot_status = RobotStatus.ESTOP
        elif is_collision(device_id):
            robot_status = RobotStatus.COLLISION
        elif is_normal(device_id):
            robot_status = RobotStatus.NORMAL
        else:
            robot_status = RobotStatus.UNKNOWN
    except (IOError, rospy.exceptions.ROSException):
        return RobotStatus.PLC_CONNECTION_ERROR
    except Exception:
        return RobotStatus.UNKNOWN
    else:
        return robot_status


def set_latest_vision_image_link(
    static_folder: str,
    link_pathname: str,
    base_log_path: str = "/home/xyz/xyz_log",
    date_format: str = "%Y%m%d",
) -> Optional[str]:
    """设置最新的视觉图片软链接，视觉图片文件名称的构成方式为
        【时间戳_相机编号_数字_图片类型.图片格式】
    比如，一张离线混码项目中的拍摄的照片数据
        【1662604407150_207000128950_2_viz.jpg】

    - 时间戳为相机拍照的时间，13位数字
    - 相机编号就是拍照相机的内置数字型号
    - 图片类型包含viz、rgb、aligned_depth等
    - 数字（暂时不清楚具体作用）
    - 图片格式为png或者jpg

    Args:
        static_folder： str, 静态资源文件的存储路径
        link_pathname: str, 软链的指向路径名
        base_log_path: str, （可选）视觉图像的日志文件保存路径
        date_format: str, （可选）视觉图像文件日期格式

    Returns:
        Optional[str]: 如果设置成功，返回原文件地址
    """
    # 读取今天拍照过的图片列表
    today = datetime.date.today().strftime(date_format)
    src = None
    for item in os.listdir(base_log_path):
        if item.startswith(today):
            src = os.path.join(base_log_path, item, "debug")
            break
    else:
        return
    # 创建软链，如果已存在则更新软链
    dst = os.path.join(static_folder, link_pathname)
    if os.path.islink(dst):
        os.unlink(dst)
    os.symlink(src, dst)
    return src


class RobotStatePublisher:
    """机器人状态发送类"""

    def __init__(self):
        self.process: Optional[subprocess.Popen] = None
        self.ros_bridge_launch_file: Optional[str] = None

    def run(self):
        """启动机器人状态发布程序"""
        with Pool(1) as pool:
            # 使用单独的进程获取配置文件
            # 因为xyz_motion这个库加载后，程序退出信号将强制变为 abort 终止信号
            self.ros_bridge_launch_file = pool.apply(
                self.get_ros_bridge_launch_file,
                args=(os.path.abspath(apps.__file__), settings),
            )
        self.process = self._start()

    def close(self):
        """关闭机器人状态发布程序"""
        if self.process:
            hmi_log.info("RobotStatePublisher is closing.")
            self.process.send_signal(signal.SIGINT)
            self.process = None
            hmi_log.info("RobotStatePublisher closed.")

    @staticmethod
    def get_ros_bridge_launch_file(app_path, settings: "Settings"):
        """Set up robot params, set static robot file"""
        try:
            from xyz_motion import RobotDriver

            robt_driver = RobotDriver(0)
            res = robt_driver.get_robot_model()
            absolute_path = res["robot_model_support"]
            launch_file = res["robot_model_launch_path"]

            uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
            roslaunch.configure_logging(uuid)
            launch = roslaunch.parent.ROSLaunchParent(uuid, [launch_file])
            launch.start()

            robot_strlist = absolute_path.split("/")
            robot_model_0 = robot_strlist[len(robot_strlist) - 1]
        except ImportError as err:
            raise XYZConfigError("xyz_motion模块导入失败，请检查是否安装了xyz_motion模块") from err
        except Exception as exc:
            from rospkg import RosPack

            outside_log.warning("通过 xyz_motion 获取机器人模型失败, {}".format(exc))
            outside_log.warning("尝试使用 config.json 中的默认机器人模型, 你可以在 config.json 中修改")

            if not settings.ROBOT_MODEL:
                raise XYZConfigError("没有设置默认机器人模型, 请在 config.json 中设置")

            with warnings.catch_warnings():
                warnings.simplefilter(action='default', category=DeprecationWarning)
                warnings.warn(
                    category=DeprecationWarning,
                    message="'ROBOT_MODEL' 已弃用，将在未来版本中删除. 建议将 xyz-palletize 升级到 0.11.0 及以上版本."
                )

            rp = RosPack()
            absolute_path = rp.get_path(settings.ROBOT_MODEL[0])
            launch_file = (
                f"{absolute_path}/launch/load_{settings.ROBOT_MODEL[1]}.launch"
            )
            uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
            roslaunch.configure_logging(uuid)
            launch = roslaunch.parent.ROSLaunchParent(uuid, [launch_file])
            launch.start()
            robot_model_0 = settings.ROBOT_MODEL[0]

        # Copy urdf
        if not os.path.exists(settings.STATIC_FOLDER):
            os.makedirs(settings.STATIC_FOLDER)
        copy_tree(absolute_path, os.path.join(settings.STATIC_FOLDER, robot_model_0))

        # Run robot state publisher
        return f"{os.path.dirname(app_path)}/rosbridge.launch"

    def _start(self) -> subprocess.Popen:
        p = subprocess.Popen(
            ["roslaunch", self.ros_bridge_launch_file],
            bufsize=0,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.STDOUT,
        )
        hmi_log.info(f"RobotStatePublisher started, PID: {p.pid}")
        return p


def to_camel_case(snake_str):
    """foo_bar -> fooBar"""
    first, *others = snake_str.split("_")
    return "".join([first.lower(), *map(str.title, others)])


def to_pascal_case(snake_str):
    """foo_bar -> FooBar"""
    return snake_str.replace("_", " ").title().replace(" ", "")
