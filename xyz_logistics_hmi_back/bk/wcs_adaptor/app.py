# -*- coding: utf-8 -*-
import importlib
import os.path
import sys
import warnings
from pathlib import Path

from flask_migrate import upgrade

from apps import _, Application, settings, openapi

# from apps.utils.validation import validate_custom_hmi_api_existed

from .helpers import load_task
from .manager import order_manager, task_manager, workspace_manager


def init_app(app: Application):
    """initialize wcs_adaptor app."""
    common_bps = get_common_blueprints()

    if len(sys.argv) > 1 and "-pp" in sys.argv:
        warnings.warn("已弃用, -pp参数将在1.3.0版本移除.")
        raise RuntimeError("请修改default.json文件, 以运行pp项目.")
    elif settings.PROJECT_TYPE == "pp":
        wcs_bps = get_pp_blueprints()
    elif settings.PROJECT_TYPE == "dpt":
        # 拆码垛
        wcs_bps = get_dpt_blueprints()

        if settings.HEARTBEAT_ENABLE:
            # 开启心跳检测
            from wcs_adaptor.depalletize.wcs import start_heartbeat_check_thread

            start_heartbeat_check_thread()
    elif settings.PROJECT_TYPE == "ind":
        wcs_bps = get_ind_blueprints()
    else:
        raise ValueError("错误的项目类型(仅支持dpt/pp/ind), 请检查default.json配置是否正确.")

    if settings.PLC_THREAD_ENABLE:
        # 开启PLC监听
        from wcs_adaptor.plc_listener import start_plc_listener_thread

        start_plc_listener_thread()

    if settings.ROBOT_STATUS_THREAD_ENABLE:
        # 开启机器人状态线程监听
        from wcs_adaptor.plc_listener import start_robot_status_thread

        start_robot_status_thread()

    # 加载蓝图
    app.register_blueprints(common_bps + wcs_bps)

    # 加载重写的视图函数
    importlib.import_module("wcs_adaptor.override_views")

    # 初始化数据库
    with app.app_context():
        # automatic upgrade db.
        upgrade(
            directory=os.path.join(
                Path(os.path.realpath(__file__)).parent, "migrations"
            )
        )
        # load backup tasks
        load_task()

    app.order_manager = order_manager
    app.task_manager = task_manager
    app.workspace_manager = workspace_manager
    # NOTICE: 这里更新v1.3.0更新辅助功能后，改动了配置结构，暂时不再进行
    # 校验，后续需要补充格式的有效性校验
    # validate_custom_hmi_api_existed(app)

    openapi.init_app(
        app,
        title="{} - XLHB API".format(settings.PROJECT_TYPE.upper()),
        version=settings.VERSION,
        host="127.0.0.1:{}".format(settings.HTTP_ADDR.port),
    )


def get_common_blueprints():
    """加载公共蓝图"""
    from .api.customized_hmi import custom_hmi_bp
    from .api.order import bp as order_bp
    from .api.task import bp as task_bp
    from .api.lowcode import bp as lowcode_bp
    from .api.grating import sbp as grating_bp

    return [custom_hmi_bp, order_bp, task_bp, lowcode_bp, grating_bp]


def get_pp_blueprints():
    """加载分拣站专有蓝图"""
    from .piece_picking.xtf import bp as xtf_bp
    from .piece_picking.wcs import bp as wcs_bp

    return [xtf_bp, wcs_bp]


def get_dpt_blueprints():
    """加载拆码垛专有蓝图接口。"""
    from wcs_adaptor.depalletize.rafcon import bp as rafcon_bp
    from wcs_adaptor.depalletize.wcs import bp as wcs_bp

    return [rafcon_bp, wcs_bp]


def get_ind_blueprints():
    """加载工业项目专有蓝图"""
    # TODO: 增加工业项目需要的蓝图，目前暂时没有
    return []
