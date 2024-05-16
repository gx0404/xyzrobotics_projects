from pathlib import Path
from typing import Optional

from apps.enums import EventType


class XYZBaseError(Exception):
    code = -1  # 默认-1, 表示异常, 在返回响应时调用
    name = "XYZ Error"  # 异常名
    error_code = 10000  # 预留, 可以为每一个异常设计异常状态码
    error_message: Optional[str] = None  # 预留, 子类重写此属性, 用于解释异常
    msg_event: Optional[EventType] = None  # 消息事件

    def __init__(
        self, error_message: Optional[str] = None, msg_event: Optional[EventType] = None
    ):
        if error_message is not None:
            self.error_message = error_message
        if msg_event is not None:
            self.msg_event = msg_event

    # TODO: 异常状态码没有确定, 此方法暂时搁置, 暂时使用name类属性指定异常名
    # @property
    # def name(self):
    #     """根据error_code返回异常名"""
    #     return ERROR_CODES.get(self.error_code, "Unknown Error")

    def set_msg_event(self, event: EventType):
        """设置消息事件.

        Args:
            event(EventType): 事件枚举值.

        """
        self.msg_event = event
        return self

    def disable_msg_event(self):
        """禁用消息事件
        
        Examples:
            >>> raise XYZBaseError().disable_msg_event()
        """
        self.msg_event = None
        return self

    def __str__(self):
        error_code = self.error_code if self.error_code is not None else "???"
        return f"< error_code: {error_code}, name: {self.name}, message: {self.error_message} >"

    def __repr__(self):
        error_code = self.error_code if self.error_code is not None else "???"
        return f"< error_code: {error_code}, name: {self.name}, message: {self.error_message} >"


class XYZIntegrityError(XYZBaseError):
    error_code = 10500
    name = "Integrity Error"  # 常用于数据库插入唯一键冲突时


class XYZCustomAPIMissingError(XYZBaseError):
    error_code = 10501
    name = "Custom API Missing"

    def __init__(self, route: str):
        self.error_message = f'Need to create route "{route}" in api/customized_hmi.py'


class XYZConfigError(XYZBaseError):
    """配置文件错误"""

    error_code = 10502
    name = "Config Error"

    def __init__(self, error_message: str):
        self.error_message = error_message


class XYZConfigParseError(XYZConfigError):
    """配置文件解析错误"""

    error_code = 10503
    name = "Config Parse Error"

    def __init__(self, filename: str, path: Path):
        self.error_message = (
            f"Failed to parse configuration file({filename}) in {path}."
        )


class XYZConfigValueError(XYZConfigError):
    """配置文件值错误"""

    error_code = 10504
    name = "Config Value Error"

    def __init__(self, filename: str, path: Path, key: str, value: str):
        self.error_message = f"Invalid value({value}) for key({key}) in configuration file({filename}) in {path}."


class XYZConfigMissingError(XYZConfigError):
    """配置文件缺失错误"""

    error_code = 10505
    name = "Config Missing Error"

    def __init__(self, filename: str, path: Path):
        self.error_message = f"Configuration file({filename}) is missing in {path}."


class XYZNotFoundError(XYZBaseError):
    error_code = 10404
    name = "Record Not Found"


class XYZUnknownSearchOperator(XYZBaseError):
    error_code = 10401
    name = "Unknown Search Operator."


class XYZValidationError(XYZBaseError):
    error_code = 10400
    name = "Validation Error"
    msg_event = EventType.ORDER_INFO


class XYZUnsupportedMediaType(XYZBaseError):
    error_code = 10415
    name = "Unsupported media type"

    def __init__(self, request_cont_type: str):
        self.error_message = f"Unsupported media type '{request_cont_type}' in request. 'application/json' is required."
