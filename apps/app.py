# -*- coding: utf-8 -*-
"""
Copyright (c) XYZ Robotics Inc. - All Rights Reserved
Unauthorized copying of this file, via any medium is strictly prohibited
Proprietary and confidential
Author: Yuhang Wu <yuhang.wu@xyzrobotics.ai>, 2022-05-04
"""
import logging
import logging.config
import queue
import threading
from typing import List, Optional, Type, TYPE_CHECKING

from flask import Blueprint, Flask
from flask_cors import CORS
from flask_restful import Api, Resource

try:
    from flask_babel import Babel
except ImportError as e:
    import sys

    # WARN: 当全局环境中没有该库时，则从虚拟环境中尝试导入
    sys.path.insert(
        0,
        "/home/xyz/.virtualenvs/xyz-logistics-hmi-back/lib/python3.8/site-packages/",
    )

    from flask_babel import Babel


from apps import error_handlers
from apps.enums import PlcStatus, RobotStatus
from apps.ext.message_push import MessagePush
from apps.exceptions import XYZBaseError
from apps.log import LOG_DEFAULT_CONF, hmi_log
from apps.models import db, migrate
from apps.services.message_push import HMIMessagePusher
from apps.settings import settings, CommonSettings
from apps.utils.log_clear import log_clear
from apps.utils.requesting.request_by_ws import Requesting
from apps.utils.upload_manager import register_upload

if TYPE_CHECKING:
    from apps.ext.manager import TaskManager
    from apps.ext.manager import OrderManager
    from apps.ext.manager import WorkspaceManager

app: "Application" = None  # type: ignore


def create_app(config=None, testing=False) -> "Application":
    """Create a new instance of Flask app.

    Args:
        config(dict):
        testing(bool): default False.

    Returns:

    """
    global app
    if app:
        return app

    # clear expired log
    log_clear.run()

    app = Application()
    # Setting up logging as early as possible
    app.config["LOG_DEFAULT_CONF"] = LOG_DEFAULT_CONF
    logging.config.dictConfig(app.config["LOG_DEFAULT_CONF"])
    # disable logger of flask app.
    log = logging.getLogger("werkzeug")
    log.setLevel(logging.ERROR)
    app.config["SQLALCHEMY_DATABASE_URI"] = settings.SQLALCHEMY_DATABASE_URI
    app.config[
        "SQLALCHEMY_TRACK_MODIFICATIONS"
    ] = settings.SQLALCHEMY_TRACK_MODIFICATIONS
    app.config["UPLOAD_FOLDER"] = settings.UPLOAD_FOLDER
    # 禁用文件缓存
    app.config["SEND_FILE_MAX_AGE_DEFAULT"] = 0
    app.config["JSON_AS_ASCII"] = False
    app.config["TESTING"] = testing
    if config:
        app.config.from_mapping(config)
    CORS(app)

    app.config["BABEL_DEFAULT_LOCALE"] = settings.LANGUAGE.name
    app.config["BABEL_TRANSLATION_DIRECTORIES"] = "{}/static/i18n/translations".format(
        settings.GENERAL_CONFIG_DIR.parent
    )
    app.config["BABEL_DOMAIN"] = settings.PROJECT_TYPE
    Babel(app)

    # init db
    db.init_app(app)
    migrate.init_app(app, db)
    # init error handlers
    error_handlers.init_app(app)
    # register blueprint or resource.
    _register_routers(app)
    # add common settings up
    settings.common_settings = CommonSettings()
    # add dpt settings up 
    if settings.PROJECT_TYPE == "dpt":
        from apps.settings import DPTSettings

        settings.dpt_settings = DPTSettings()
    return app


def _register_routers(app: "Application") -> None:
    """注册路由."""
    from apps import base_app

    base_app.init_app(app)

    # ext 下的视图涉及导入 wcs_adaptor 模块，因此函数内部导入避免循环导入异常
    from apps.ext.views import bp as ext_bp

    app.register_blueprint(ext_bp)

    # if settings.PROJECT_TYPE == "dpt":
    # TODO(YuhangWu): 按项目类型注册路由
    #     dpt.bp_list 中的路由应该仅属于 dpt 项目
    from apps.programs.dpt.views.design_v2 import design_bps
    from apps.programs.dpt import bp_list

    app.register_blueprints(bp_list + design_bps)
    register_upload(app)


def cached_app():
    """在创建app之后, 通过此方法获取全局唯一的app实例.

    虽然调用create_app也可以到达同样的效果, 为了避免歧义, 则获取已创建的app均调用此方法.

    Returns:
        Application: an instance of Application.
    """
    if app:
        return app
    raise XYZBaseError(
        error_message="the app must be created before call this function."
    )


def create_socketio(flask_app=None):
    """Create a new instance of SocketIO.

    Args:
        flask_app(Flask): an instance of Flask app.

    Returns:

    """
    from flask_socketio import SocketIO
    from apps.base_app.ws_event import Connection

    socketio = SocketIO(flask_app, cors_allowed_origins="*", async_mode="threading")
    socketio.on_namespace(Connection("/", socketio))
    return socketio


class Globals:
    """全局对象.

    用于存储全局变量.
    可用于多线程中, 通过此对象获取全局变量.
    内部使用了 threading.Rlock 保证线程安全.
    """

    __lock: threading.RLock
    _rafcon_error: queue.Queue
    _plc_status: PlcStatus
    _robot_status: RobotStatus

    def __init__(self):
        # 可重入锁
        object.__setattr__(self, "_Globals__lock", threading.RLock())
        # rafcon 异常
        object.__setattr__(self, "_rafcon_error", queue.Queue(maxsize=1))
        # plc 状态
        object.__setattr__(self, "_plc_status", PlcStatus.STOPPED)
        # 机器人状态
        object.__setattr__(self, "_robot_status", RobotStatus.STOPPED)

    @property
    def plc_status(self):
        """plc 状态."""
        return self._plc_status

    @plc_status.setter
    def plc_status(self, value: PlcStatus):
        self._plc_status = value

    @property
    def robot_status(self):
        """机器人状态."""
        return self._robot_status

    @robot_status.setter
    def robot_status(self, value: RobotStatus):
        self._robot_status = value

    @property
    def rafcon_error(self):
        """rafcon 异常."""
        return self._rafcon_error

    @rafcon_error.setter
    def rafcon_error(self, value):
        """设置rafcon异常."""
        self.set_rafcon_error(value)

    def set_rafcon_error(self, value):
        """设置 rafcon 异常.

        如果队列已满, 则丢弃旧的异常, 保留新的异常.
        """
        try:
            self._rafcon_error.put_nowait(value)
        except queue.Full:
            error = self._rafcon_error.get_nowait()
            self._rafcon_error.put_nowait(value)
            hmi_log.error(
                "rafcon error queue is full, drop the oldest error. '%s'", error
            )

    def get_rafcon_error(self):
        """获取 rafcon 异常并清空.

        Raises:
            queue.Empty: 队列为空.
        """
        try:
            return self._rafcon_error.get_nowait()
        except queue.Empty:
            return None

    def __getitem__(self, item):
        return getattr(self, item)

    def __setitem__(self, key, value):
        setattr(self, key, value)

    def __setattr__(self, key, value):
        """设置属性."""
        with self.__lock:
            return super().__setattr__(key, value)


class Application(Flask):
    task_manager: "TaskManager"
    order_manager: "OrderManager"
    workspace_manager: "WorkspaceManager"

    def __init__(
        self,
        static_url_path=None,
        static_folder=settings.STATIC_FOLDER,
        static_host=None,
        host_matching=False,
        subdomain_matching=False,
        template_folder="templates",
        instance_path=None,
        instance_relative_config=False,
        root_path=None,
    ):
        super().__init__(
            __name__,
            static_url_path,
            static_folder,
            static_host,
            host_matching,
            subdomain_matching,
            template_folder,
            instance_path,
            instance_relative_config,
            root_path,
        )
        self.flask_api = Api(self)
        self.socketio = create_socketio(self)
        self.requesting = Requesting(self.socketio)
        self.hmi_msg_pusher = HMIMessagePusher(self.socketio)
        self.mp = MessagePush(self.socketio)
        try:
            from xyz_central_hub.client import HubClient

            self.hub_client = HubClient()
        except ImportError:
            self.hub_client = None

        # 存放全局唯一的属性
        self.globals = Globals()
        self.status = "stopped"
        self.mode = "robot"
        self.plc_light = [0, 0, 0]  # [green, yellow, red]

    def register_blueprints(self, blueprints: List[Blueprint]):
        """批量注册蓝图.

        Args:
            blueprints(List[Blueprint]): 蓝图的列表集合.

        """
        for bp in blueprints:
            self.register_blueprint(bp)

    def register_resource(self, resource: Type[Resource], *url, **kwargs):
        """Wrap api.add_resource,
        make it possible to ignore register a route repeatedly.

        Args:
            resource: an instance of flask_restful.Resource.
            *url:
            **kwargs:

        Returns:

        """
        ignore_repeat = kwargs.pop("ignore_repeat", False)
        try:
            self.flask_api.add_resource(resource, *url, **kwargs)
        except ValueError as e:
            if not ignore_repeat:
                raise e
            hmi_log.warn("And app choose to ignore it and give up this register")

    def override_view(self, path, **options):
        """Override view function of a route."""
        def decorator(f):
            endpoint = options.pop("endpoint", None)
            # remove the old rule
            for rule in self.url_map.iter_rules():
                if rule.rule == path:
                    if endpoint is None or rule.endpoint == endpoint:
                        self.url_map._rules.remove(rule)
                        self.view_functions.pop(endpoint)
                        break
                    else:
                        raise ValueError("路径已经存在, 但是 endpoint 不同，无法覆盖")
            # add the new rule
            self.add_url_rule(path, endpoint, f, **options)
            return f

        return decorator

    def run_by_socketio(
        self,
        host: str = settings.HTTP_ADDR.host,
        port: int = settings.HTTP_ADDR.port,
        debug: bool = settings.DEBUG,
    ):
        """通过socketio的方式启动程序."""
        self.socketio.run(
            self,
            host=host,
            port=port,
            debug=debug,
        )
