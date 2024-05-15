"""
系统设置
"""
import base64
import contextlib
import json
import os
import os.path
import re
import sys
import warnings
from distutils.dir_util import copy_tree
from pathlib import Path
from typing import Any, Dict, List, Optional, Literal, cast
from typing_extensions import TypedDict

from pydantic import (
    AnyUrl,
    BaseModel,
    BaseSettings,
    Extra,
    Field,
    validator,
    root_validator,
)

from apps.enums import Language
from apps.exceptions import XYZBaseError, XYZConfigError, XYZConfigMissingError

APP_PATH = Path(os.path.abspath(__file__)).parent


def json_config_settings_source(s: BaseSettings) -> Dict[str, Any]:
    """加载JSON配置文件.

    Args:
        s: 配置对象.

    Returns:
        dict: 字典对象.

    Raises:
        XYZConfigMissingError: 配置文件加载失败.
    """

    encoding = s.__config__.env_file_encoding
    file_path = s.__config__.config_file_path
    if not os.path.exists(file_path):
        raise XYZConfigMissingError(filename=file_path.name, path=file_path)
    return json.loads(file_path.read_text(encoding))


class MySQLDsn(AnyUrl):
    """MySQL DSN"""

    allowed_schemes = {"mysql+pymysql"}
    user_required = True


class Socket(BaseModel):
    host: str = Field(description="IP地址")
    port: int = Field(description="端口号")

    def __str__(self):
        return f"{self.host}:{self.port}"

    def __repr__(self):
        return f"[Socket] {self.__str__()}"


class ExtraColumn(BaseModel):
    """额外的列"""

    key: str = Field(None, description="键名")
    name: str = Field(description="列名")
    enable: bool = Field(True, description="是否启用")


class OfflineMixPlannerConfig(BaseSettings):
    """任务规划"""

    max_workers: int = Field(default=1, description="同时进行的最大规划任务数")
    timeout: Optional[int] = Field(
        default=None, title="单个规划的超时时间", description="单位秒, 默认为None, 表示不限制超时时间"
    )


class SkuLimit(BaseModel):
    min_length: float = Field(default=0, ge=0, le=99999)
    max_length: float = Field(default=99999, ge=0, le=99999)
    min_width: float = Field(default=0, ge=0, le=99999)
    max_width: float = Field(default=99999, ge=0, le=99999)
    min_height: float = Field(default=0, ge=0, le=99999)
    max_height: float = Field(default=99999, ge=0, le=99999)
    min_weight: float = Field(default=0, ge=0, le=99999)
    max_weight: float = Field(default=99999, ge=0, le=99999)

    @validator("max_length")
    def validate_length(cls, v: float, values: dict) -> float:
        if v < values["min_length"]:
            raise XYZConfigError("SKU长度限制的最小长度{min_length}超过最大长度{max_length}，长度限制校验失败！".format(
                min_length=values["min_length"],
                max_length=v
            ))
        return v

    @validator("max_width")
    def validate_width(cls, v: float, values: dict) -> float:
        if v < values["min_width"]:
            raise XYZConfigError("SKU宽度限制的最小宽度{min_width}超过最大宽度{max_width}，宽度限制校验失败！".format(
                min_width=values["min_width"],
                max_width=v
            ))
        return v

    @validator("max_height")
    def validate_height(cls, v: float, values: dict) -> float:
        if v < values["min_height"]:
            raise XYZConfigError("SKU高度限制的最小高度{min_height}超过最大高度{max_height}，高度限制校验失败！".format(
                min_height=values["min_height"],
                max_height=v
            ))
        return v

    @validator("max_weight")
    def validate_weight(cls, v: float, values: dict) -> float:
        if v < values["min_weight"]:
            raise XYZConfigError("SKU重量限制的最小重量{min_weight}超过最大重量{max_weight}，重量限制校验失败！".format(
                min_weight=values["min_weight"],
                max_weight=v
            ))
        return v


class CommonSettings(BaseModel):
    """项目通用配置."""
    SKU_LIMIT: SkuLimit = Field(
        default_factory=SkuLimit,
        description="sku长宽高重大小限制",
        alias="sku_limit"
    )

    @root_validator(pre=True)
    def root_validate(cls, values: dict) -> dict:
        config_path = settings.Config.config_path / "common.json"
        if not os.path.exists(config_path):
            with open(config_path, "w", encoding="utf-8") as f:
                default_config = {
                    "sku_limit": SkuLimit().dict()
                }
                json.dump(default_config, f, ensure_ascii=False, indent=4)
            return {}
        with open(config_path, "r", encoding="utf-8") as f:
            t = f.read()
            return json.loads(t)

    def dumps(self):
        """持久化配置文件."""
        data = self.json(
            exclude=self.__exclude_fields__,
            by_alias=True,
            indent=4
        )
        # /home/xyz/xyz_app/app/xyz_logistics_hmi_back/config
        config_path = settings.Config.config_path / "common.json"
        with open(config_path, "w", encoding="utf-8") as f:
            f.write(data)


class DPTSettings(BaseModel):
    """拆码垛配置."""

    extra_columns_for_register_box: Dict[
        Literal["extra1", "extra2", "extra3", "extra4"], ExtraColumn
    ] = Field(default_factory=dict, description="注册纸箱时, 额外的字段")
    offline_mix_planner: OfflineMixPlannerConfig = Field(
        default_factory=OfflineMixPlannerConfig,
        description="任务规划",
    )

    @root_validator(pre=True)
    def root_validate(cls, values: dict) -> dict:
        config_path = settings.Config.config_path / "dpt.json"
        if not os.path.exists(config_path):
            with open(config_path, "w", encoding="utf-8") as f:
                default_config = {
                    "extra_columns_for_register_box": {
                        "extra1": {"name": "扩展字段1", "key": "extra1", "enable": False},
                        "extra2": {"name": "扩展字段2", "key": "extra2", "enable": False},
                        "extra3": {"name": "扩展字段3", "key": "extra3", "enable": False},
                        "extra4": {"name": "扩展字段4", "key": "extra4", "enable": False},
                    }
                }
                json.dump(default_config, f, ensure_ascii=False, indent=4)
            return {}
        with open(config_path, "r", encoding="utf-8") as f:
            t = f.read()
            return json.loads(t)


class PLCAddress(TypedDict, total=False):
    grating: int  # 光栅
    buzzer: int   # 蜂鸣器
    red: int      # 红灯
    yellow: int   # 黄灯
    green: int    # 绿灯
    estop: int    # 急停


class Settings(BaseSettings):
    """
    统一配置类
    """

    __exclude_fields__ = {
        "LOGO",
        "VERSION",
        "WCS_IP",
        "WCS_PORT",
        "HTTP_ADDR",
        "DB_PSWD",
        "PROJECT_NAME",
        "dpt_settings",
        "common_settings"
    }
    PROJECT_TYPE: str = Field(default="dpt", description="项目类型", alias="project_type")
    VERSION: str = Field(default=None, description="系统版本", alias="version")

    @validator("VERSION")
    def make_version(cls, *args) -> str:
        """从CHANGELOG中解析最新的版号."""
        path = Path(os.path.realpath(__file__)).parent.parent
        # 使用xlhb命令启动程序时，使用的是site-packages/apps这个包
        # 为了能拿到CHANGELOG.md以获取版本号，需要向上跳转两级
        # 去获取到site-packages/xyz_logistics_hmi_back目录下的CHNAGELOG.md
        if "site-packages" in str(path) \
                and "xyz_logistics_hmi_back" not in str(path):
            path = os.path.join(path, "xyz_logistics_hmi_back")

        with open(os.path.join(path, "CHANGELOG.md"), "r", encoding="utf-8") as f:
            text = f.readline()

        try:
            return re.findall(r"(?<=## ).*(?=\()", text)[0]
        except IndexError:
            raise XYZBaseError(
                error_message="occur error during parse version information."
            ) from None

    PROJECT_NAME: Optional[str] = Field(
        default=None, description="项目名称", alias="project_name"
    )

    @validator("PROJECT_NAME")
    def make_project_name(cls, v: Optional[str]) -> str:
        """获取当前运行项目的名称."""
        # 读取环境变量的PS1
        ps1 = os.getenv("PS1")

        if os.getenv("XLHB_DEV") == "1" or ps1 is None:
            v = "xlhb_dev"
        else:
            # 从PS1环境变量中获取当前项目的名称.
            match = re.search(r"\((\w+)\)", ps1)
            v = "xlhb_dev" if match is None else match.groups()[0]
        return v

    LANGUAGE: Language = Field(
        default=Language.zh, description="系统语言", alias="language"
    )
    HOME: Path = Field(..., env="HOME", description="工作目录", alias="home")
    CODE_BASE: Path = Field(..., env="CODE_BASE", description="代码目录", alias="code_base")
    LOGO: Optional[str] = Field(default=None, description="logo图片base64")
    LOGO_FILENAME: Optional[str] = Field(
        default=None, description="logo文件名", alias="logo_filename"
    )
    DEBUG: bool = Field(default=False, description="是否为DEBUG模式", alias="debug")
    DEFAULT_PALLET_ID: int = Field(
        default=1, description="默认托盘ID", alias="default_pallet_id"
    )
    SUPERVISOR_PORT: int = Field(
        default=9001, description="Supervisor端口号", alias="supervisor_port"
    )
    UPLOAD_FOLDER: Path = Field(description="保存上传文件的目录", alias="upload_folder")
    RAFCON_DEBUG: bool = Field(description="是否开启rafcon的DEBUG模式", alias="rafcon_debug")
    STOP_ROBOT_IF_ERROR: bool = Field(
        default=True,
        description="当系统出现异常时，是否自动关闭机器人节点",
        alias="stop_robot_if_error",
    )
    STOP_ROBOT_IF_STOP: bool = Field(
        default=True,
        description="当系统停止后，是否自动关闭机器人节点",
        alias="stop_robot_if_stop",
    )
    STATE_MACHINE_PATH: List[Path] = Field(alias="state_machine_path")
    HEARTBEAT_TIMEOUT: int = Field(description="心跳周期", alias="heartbeat_timeout")
    HEARTBEAT_ENABLE: bool = Field(
        default=False, description="是否开启心跳检测", alias="heartbeat_enable"
    )
    PLC_THREAD_ENABLE: bool = Field(
        default=False, description="是否开启pld线程", alias="plc_thread_enable"
    )
    PLC_DEVICE_ID: str = Field(
        default="1", description="PLC设备ID", alias="plc_device_id"
    )
    PLC_ADDRESS_DICT: PLCAddress = Field(
        description="plc address dict",
        alias="plc_address_dict"
    )
    # 是否启用机器人状态线程
    ROBOT_STATUS_THREAD_ENABLE: bool = Field(
        default=False, description="是否开启监听机器人状态线程", alias="robot_status_thread_enable"
    )
    # 前端锁屏密码
    SCREEN_LOCK_PASSWORD: str = Field(
        default="123456", description="前端应用程序的锁屏解锁密码", alias="screen_lock_password"
    )
    # 数据库配置
    DB_PSWD: Optional[str] = Field(default=None, description="数据库密码", alias="db_pswd")
    SQLALCHEMY_TRACK_MODIFICATIONS: bool = Field(
        default=False, alias="sqlalchemy_track_modifications"
    )
    SQLALCHEMY_DATABASE_URI: MySQLDsn = Field(
        description="SQLAlchemy数据库连接", alias="sqlalchemy_database_uri"
    )

    @validator("SQLALCHEMY_DATABASE_URI")
    def make_sqlalchemy_database_uri(cls, v: MySQLDsn) -> MySQLDsn:
        """生成数据库连接."""
        # 优先从环境变量中获取数据库连接信息.
        if uri := os.environ.get("SQLALCHEMY_DATABASE_URI"):
            return MySQLDsn(uri, scheme="mysql+pymysql")
        return v

    GENERAL_CONFIG_DIR: Path = Field(default=None, alias="general_config_dir")

    @validator("GENERAL_CONFIG_DIR")
    def make_config_dir(cls, v: Optional[Path], values: Dict) -> Path:
        # if v is None:
        #     return os.path.join(
        #         values["CODE_BASE"],
        #         "app/xyz_logistics_hmi_back/config"
        #     )
        # return v
        return cls.Config.config_path

    # FIXME(YuhangWu): 当Config路径改变，而与此相关的配置文件路径并没有随之改变
    #   修改建议： 辅助接口的配置文件路径，最好是禁止单独配置。
    #   所有配置文件都应在config目录下，所以仅可配置config目录，
    #   而具体的文件位置，则使用property的方式计算得到即可。
    CMCT_LOG_PATH: Path = Field(default=None, alias="cmct_log_path")

    @validator("CMCT_LOG_PATH")
    def make_log_path(cls, v: Optional[str], values: Dict) -> str:
        if v is None:
            return os.path.join(values["HOME"], "xyz_log/xyz_logistics_hmi_back")
        return v

    # 地址配置
    WCS_IP: str = Field(default="0.0.0.0", description="wcs的ip地址", alias="wcs_ip")
    WCS_PORT: int = Field(default=12000, description="wcs的端口号", alias="wcs_port")
    WCS_ADDR: Socket = Field(default=None, description="WCS通信地址", alias="wcs_addr")

    @validator("WCS_ADDR")
    def make_wcs_addr(cls, v: Socket, values: Dict) -> Socket:
        if v is None:
            return Socket(host=values.get("WCS_IP"), port=values.get("WCS_PORT"))
        else:
            values["WCS_IP"] = v.host
            values["WCS_PORT"] = v.port
            return v

    SOCK_ADDR: Socket = Field(
        default=Socket(host="127.0.0.1", port=7001), alias="sock_addr"
    )
    XYZ_HTTP_PORT: int = Field(
        default=7002, description="HTTP接口的端口号", alias="xyz_http_port"
    )
    HTTP_ADDR: Socket = Field(
        default=Socket(host="0.0.0.0", port=7002), description="后端地址", alias="http_addr"
    )

    @validator("HTTP_ADDR")
    def make_http_addr(cls, v: Socket, values: Dict) -> Socket:
        xyz_http_port = values.get("XYZ_HTTP_PORT")
        if xyz_http_port != 7002:
            v.port = xyz_http_port
        return v

    ROBOT_MODEL: Optional[list] = Field(
        default=None, description="ROBOT_MODEL", alias="robot_model"
    )

    @validator("ROBOT_MODEL")
    def make_robot_model(cls, v: Optional[list]) -> Optional[list]:
        with warnings.catch_warnings():
            warnings.simplefilter(
                action='default',
                category=DeprecationWarning
            )
            warnings.warn(
                category=DeprecationWarning,
                message="'ROBOT_MODEL' 已弃用，将在未来版本中删除. 建议将 xyz-palletize 升级到 0.11.0 及以上版本.",
            )
        return v

    TOPIC_LIST: list = Field(
        default=[
            "/real_planning_environment_marker_topic",
            "/booked_items_marker_topic",
        ],
        title="订阅列表",
        description="用于HMI显示3D动画的topic列表",
        alias="topic_list",
    )
    STATIC_FOLDER: str = Field(title="静态文件目录", alias="static_folder")
    CUSTOM_HMI_CG: Path = Field(
        default=None, description="自定义API的配置文件路径", alias="custom_hmi_cg"
    )
    dpt_settings: Optional[DPTSettings] = Field(
        default=None, title="DPT配置", description="DPT配置"
    )

    common_settings: Optional[CommonSettings] = Field(
        default=None,
        title="项目通用配置",
        description="项目的常用配置和通用配置，该配置有时候需要在后端启动后被更新修改"
    )

    @validator("CUSTOM_HMI_CG")
    def make_custom_hmi_config(cls, v: Optional[Path]) -> Path:
        # if v is None:
        #     path = cls.Config.config_path
        #     v = Path(os.path.join(path, "custom_hmi.json")).absolute()
        # return v
        return Path(cls.Config.config_path, "custom_hmi.json")
    
    @property
    def LOWCODE_CONFIG_PATH(self) -> Path:
        lowcode_path = Path(self.GENERAL_CONFIG_DIR, "lowcode.json")
        if not os.path.exists(lowcode_path):
            with open(lowcode_path, "w", encoding="utf-8") as f:
                f.write("{}")
        return Path(self.GENERAL_CONFIG_DIR, "lowcode.json")

    class Config:
        env_file_encoding = "utf-8"
        config_path = Path(
            os.path.join(
                os.getenv("CODE_BASE"),
                "app/xyz_logistics_hmi_back/config/",
            )
        )
        config_file_path = Path(config_path, "default.json")
        extra = Extra.ignore

        @classmethod
        def customise_sources(
            cls,
            init_settings,
            env_settings,
            file_secret_settings,
        ):
            return (
                init_settings,
                json_config_settings_source,
                env_settings,
                file_secret_settings,
            )

    def dumps(self):
        """持久化配置文件."""
        data = self.json(exclude=self.__exclude_fields__, by_alias=True, indent=4)
        # /home/xyz/xyz_app/app/xyz_logistics_hmi_back/config
        config_path: Path = self.__config__.config_path
        os.makedirs(config_path, exist_ok=True)
        with open(
            os.path.join(config_path, "default.json"), "w", encoding="utf-8"
        ) as f:
            f.write(data)

    def load_logo(self) -> None:
        """加载logo图片"""
        if self.LOGO_FILENAME is None:
            return None
        with open(os.path.join(self.Config.config_path, self.LOGO_FILENAME), "rb") as f:
            b = f.read()
        self.LOGO = base64.b64encode(b).decode()

    def save_logo(self, file: bytes) -> None:
        """存储logo图片."""
        with open(os.path.join(self.Config.config_path, self.LOGO_FILENAME), "wb") as f:
            f.write(file)
        self.LOGO = base64.b64encode(file).decode()


flask_space = {
    "PRE_START_EVENT": [],  # made up with recall func without args
    "PRE_STOP_EVENT": [],  # made up with recall func without args
    "POST_START_EVENT": [],
    "POST_STOP_EVENT": [],
}

settings: Settings = cast(Settings, None)


def init_settings() -> Settings:
    global settings

    _start_if_new_project()
    back_path = _make_back_path()

    if settings is None:
        config_path = None

        # NOTICE: 什么是XLHB_DEV?
        # 开发环境分为两种： 1. 标准库的开发环境； 2. 系统工程师的开发环境。
        # 1. 标准库的开发环境，指的是apps的开发环境，在该环境中，wcs_adaptor, config, apps
        #   这三个目录都在统一根目录下，所以在此环境中运行后端程序，即默认加载apps同根目录下的
        #   wcs_adaptor与config
        # 2. 系统工程师的开发环境，指的是系统工程师开发adaptor的环境。
        # 判断当前运行环境是否为XLHB_DEV
        if os.environ.get("XLHB_DEV") == "1":
            config_path = Path(back_path, "config")

        if len(sys.argv) > 1 and "run" in sys.argv:
            # 从命令行中获取项目名称，并根据项目名称生成config目录.
            if "-p" in sys.argv or "--project-name" in sys.argv:
                project_name = _load_project_name()
                config_path = os.path.join(
                    os.getenv("CODE_BASE"),
                    f"projects/{project_name}/xyz_logistics_hmi_back/config/",
                )
            # 从命令行中获取config配置目录.
            if "-c" in sys.argv or "--config" in sys.argv:
                config_path = _load_config_path()

        # 刷新config配置目录.
        _refresh_config_path(config_path)

        # 实例化settings配置.
        settings = Settings()
        settings.load_logo()
        settings.dumps()

    # 如果是开发环境则设置debug为True.
    _set_debug_if_development()

    # 更新project_name.
    with contextlib.suppress(NameError):
        settings.PROJECT_NAME = project_name  # type: ignore
    return settings


def _set_debug_if_development():
    """如果为开发模式自动设置debug为True."""
    global settings
    if "--debug" in sys.argv or os.environ.get("FLASK_ENV") == "development":
        settings.DEBUG = True


def _start_if_new_project():
    """如果开启一个新项目则自动复制目录及文件."""
    global settings
    if "startproject" in sys.argv:
        # startproject命令用于创建新的后端项目, 此时配置目录下default.json文件不存在,
        # 所以在此处自动拷贝到对应目录.
        if not os.path.exists(Settings.Config.config_path):
            template_path = "/home/xyz/xyz_app/xyz_logistics_hmi_back/"
            copy_tree(
                os.path.join(template_path, "config"), str(Settings.Config.config_path)
            )
        settings = Settings()


def _make_back_path():
    """生成后端目录."""
    current_file_path = Path(os.path.abspath(__file__))
    back_path = current_file_path.parent.parent
    if os.path.exists(Path(back_path, "config")) and os.path.exists(
        Path(back_path, "wcs_adaptor")
    ):
        os.environ["XLHB_DEV"] = "1"
    return back_path


def _refresh_config_path(config_path):
    """刷新config_path的配置."""
    if config_path:
        if not os.path.exists(config_path):
            raise FileNotFoundError(f"配置文件不存在，请检查路径是否正确: {config_path}")

        Settings.Config.config_path = config_path
        Settings.Config.config_file_path = Path(config_path, "default.json")
        WCSBaseSettings.reload_config()


def _load_config_path():
    """根据项目名称获取config目录

    Returns:
        Path: configuration path.

    Raises:
        RuntimeError: config_path不能为None.
    """
    config_path = None
    if "-c" in sys.argv:
        config_path = Path(sys.argv[sys.argv.index("-c") + 1])
    elif "--config" in sys.argv:
        config_path = Path(sys.argv[sys.argv.index("--config") + 1])
    if config_path is None:
        raise RuntimeError("config_path不能为空.")
    return config_path


def _load_project_name():
    """加载命令行指定的项目名称."""
    project_name = None
    if "-p" in sys.argv:
        project_name = Path(sys.argv[sys.argv.index("-p") + 1])
    elif "--project-name" in sys.argv:
        project_name = Path(sys.argv[sys.argv.index("--project-name") + 1])
    if project_name is None:
        raise RuntimeError("project_name不能为空.")
    elif not os.path.exists(
        os.path.join(os.getenv("CODE_BASE"), f"projects/{project_name}")
    ):
        project_names = os.listdir(os.path.join(os.getenv("CODE_BASE"), "projects"))
        project_names = list(filter(lambda x: not x.startswith("."), project_names))
        raise FileNotFoundError(f"项目不存在，请检查项目名是否输入正确，当前已有项目:\n{project_names}")
    return project_name


class WCSBaseSettings(BaseSettings):
    __exclude_fields__: set = set()

    def dumps(self):
        """持久化配置文件."""
        data = self.json(exclude=self.__exclude_fields__, by_alias=True, indent=4)
        # /home/xyz/xyz_app/app/xyz_logistics_hmi_back/config
        config_path: Path = self.__config__.config_path
        file_path: Path = self.__config__.config_file_path
        os.makedirs(config_path, exist_ok=True)
        with open(file_path, "w", encoding="utf-8") as f:
            f.write(data)

    @classmethod
    def reload_config(cls):
        """重新加载WCSSettings的Config配置."""
        cls.Config.config_path = Settings.Config.config_path
        cls.Config.config_file_path = Path(
            Settings.Config.config_path, "wcs_adaptor.json"
        )

    class Config:
        env_file_encoding = "utf-8"
        config_path = Settings.Config.config_path
        config_file_path = Path(config_path, "wcs_adaptor.json")
        extra = Extra.ignore

        @classmethod
        def customise_sources(
            cls,
            init_settings,
            env_settings,
            file_secret_settings,
        ):
            return (
                init_settings,
                json_config_settings_source,
                env_settings,
                file_secret_settings,
            )
