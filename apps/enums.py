from enum import Enum


class Language(Enum):
    """语言类型."""
    zh = 1  # 汉语
    en = 2  # 英语
    ja = 3  # 日语
    de = 4  # 德语


class EventType(Enum):
    """Socket 事件."""
    ORDER_INFO = "order_info"
    MOTION_PLAN_LOG = "motion_plan_log"
    SYSTEM_LOG = "system_log"
    NODE_LOG = "receive_node_log"
    NODE_ERROR_EVENT = "node_error"
    # TODO: 使用的大驼峰命名法，之后需要前后端统一修改为下划线命名法
    ROBOT_STATUS = "RobotStatus"
    VISION = "vision_node"
    # 三色灯
    RGB_LIGHT = "rgblight"
    # WCS 连接状态
    WCS_CONNECTION = "wcs_connection"


class ProjectType(Enum):
    """项目类型."""
    DPT = dpt = "dpt"
    PP = pp = "pp"
    IND = ind = "ind"


class ErrorSources(Enum):
    """异常来源."""
    XTF = "xtf"
    VISION = "vision"


class MediaType(Enum):
    """媒体类型."""
    # content-type对应表:
    # https://blog.csdn.net/xiaoyu19910321/article/details/79279364
    xls = "application/vnd.ms-excel"
    xlsx = "application/vnd.openxmlformats-officedocument.spreadsheetml.sheet"
    png = "image/png"
    gif = "image/gif"
    jpg = "image/jpg"
    json = "application/json"
    zip = "application/zip"


class PlcStatus(Enum):
    STOPPED = "STOPPED" # 已停止
    STARTED = "STARTED" # 已启动
    EXCEPTION = "EXCEPTION" # 发生异常


class RobotStatus(Enum):
    """机器人状态

        IMPORT_IO_ERROR: 导入IO模块失败
        PLC_CONNECTION_ERROR: PLC通信失败
        STOPPED: 机器人停止
        NORMAL: 机器人状态正常
        PAUSED: 机器人运动暂停
        ESTOP: 机器人紧急停止
        ERROR: 机器人发生报错
        COLLISION: 机器人发生碰撞
        MANUAL: 机器人当前处于手动模式
        DISABLED: 机器人未处于使能状态
        NOT_CYCLE: 非循环，机器人不在循环中
        UNKNOWN: 未知的机器人状态
    """
    IMPORT_IO_ERROR = "IMPORT_IO_ERROR"
    PLC_CONNECTION_ERROR = "PLC_CONNECTION_ERROR"

    STOPPED = "STOPPED"
    NORMAL = "NORMAL"
    PAUSED = "PAUSED"
    ESTOP = "ESTOP"
    ERROR = "ERROR"
    COLLISION = "COLLISION"
    MANUAL = "MANUAL"
    DISABLED = "DISABLED"
    NOT_CYCLE = "NOT_CYCLE"
    UNKNOWN = "UNKNOWN"
