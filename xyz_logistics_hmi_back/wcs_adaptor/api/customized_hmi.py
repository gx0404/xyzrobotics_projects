# -*- coding: utf-8 -*-
"""
本页面主要用于实现辅助功能的自定义接口，辅助功能的相关配置请查看config目录中的custom_hmi.json文件，有五个演示示例。

另外，custom_hmi.json的接口地址并非一定要使用本页面中的接口，可根据具体需要灵活配置。如custom_hmi.json中的单拆接
口示例实际可使用项目接口【api/custom/example/single_class_pal_task】，而不用在本页面总再次实现一次接口。
"""
from flask import Blueprint, request

from apps import make_json_response, settings
from apps.settings import SkuLimit
from apps.base_app.flask_hook import req_log
from apps.log import wcs_log
from apps.utils.validation import validator
from wcs_adaptor.depalletize.schema import SingleTaskCreateSchema
custom_hmi_bp = Blueprint("custom", __name__, url_prefix="/api/custom")
from wcs_adaptor.helpers import backup_manager_wrapper
from wcs_adaptor.manager import order_manager, task_manager
from apps import mp,cached_app
import subprocess
from datetime import datetime
#中欧网页打开
@custom_hmi_bp.route("/example/open_html_0", methods=["POST","GET"])
@req_log(log=wcs_log)
def open_html_0():
    """通过HMI【辅助功能】界面发送请求到本接口, 打开一个网页
    本接口的路径是: api/custom/example/open_html_0

    Examples:
        {
            "code": 0,
            "msg": "success",
          }
    """
    import webbrowser
    import time
    return_data = webbrowser.open("/home/xyz/xyz_app/projects/dapeng_station_0/web_ros/simulator.html")
    time.sleep(1)
    #import ipdb;ipdb.set_trace()
    return make_json_response(msg=return_data)

@custom_hmi_bp.route("/example/clear_manager", methods=["POST"])
@req_log(log=wcs_log)
@backup_manager_wrapper()
def example_clear_manager():
    app = cached_app()
    with app.app_context():
        if app.status == "ready":
            return make_json_response(msg="正在运行程序,禁止清除任务,请停止任务后清除")      
        else:  
            """结束并清除现有的任务"""
            # task_manager.terminate(error="触发清空任务")
            # task_manager.clear()
            # order_manager.terminate(error="触发清空订单")
            # order_manager.clear()
            task = task_manager.first()
            if task:
                task.finish()
                task.end()
                task_manager.clear()
                mp.order.info("已清空任务")
            else:
                mp.order.info("无任务，无需清空任务")        
            return make_json_response(msg="已清空任务")


@custom_hmi_bp.route("/example/clear_cache_environment", methods=["POST"])
@req_log(log=wcs_log)
@backup_manager_wrapper()
def clear_cache_environment():
    app = cached_app()
    with app.app_context():
        if app.status == "ready":
            return make_json_response(msg="正在运行程序,禁止清除缓存区环境,请停止任务后清除")      
        else:  
            try:
                from xyz_env_manager.client import (
                    remove_bottom_padding_workspace,
                    clear_container_all_items,
                    clear_planned_items,
                )
                workspace_id = "2"
                remove_bottom_padding_workspace(workspace_id)
                clear_container_all_items(workspace_id)
                clear_planned_items(workspace_id)
                return make_json_response("清除缓存区环境成功")
            except ImportError as err:
                return make_json_response("失败，请检查环境节点是否打开")

@custom_hmi_bp.route("/example/code_wcs_log", methods=["POST"])
@req_log(log=wcs_log)
@backup_manager_wrapper()
def code_wcs_log():
    now_day = datetime.now().strftime("%Y-%m-%d")
    # 指定code命令的完整路径
    code_path = "/usr/share/code/bin/code" 
 
    file_path = "/home/xyz/xyz_log/xyz_logistics_hmi_back/wcs_log/"+"~"+now_day.replace("-","")+".log"
    # 使用subprocess模块的run函数执行命令
    subprocess.run([code_path, file_path])
    return make_json_response(msg="已打开日志")


@custom_hmi_bp.route("/example/code_xtf_log", methods=["POST"])
@req_log(log=wcs_log)
@backup_manager_wrapper()
def code_xtf_log():
    # 指定code命令的完整路径
    code_path = "/usr/share/code/bin/code" 
    file_path = "/home/xyz/xyz_log/xtf_log/xtf.log"
    # 使用subprocess模块的run函数执行命令
    subprocess.run([code_path, file_path])
    return make_json_response(msg="已打开日志")



@custom_hmi_bp.route("/example/sku_limit", methods=["GET"])
@req_log(log=wcs_log)
def get_sku_limit():
    """通过HMI【辅助功能】界面发送请求到本接口, 获取当前的SKU长宽高重量信息的数值，注意是GET请求
    本接口的路径是: api/custom/example/update_sku_limit

    Examples:
        {
            "code": 0,
            "msg": "success",
            "data": {
                "min_length": 0,
                "max_length": 99999,
                "min_width": 0,
                "max_width": 99999,
                "min_heightx": 0,
                "max_height": 99999,
                "min_weight": 0.01,
                "max_weight": 99999
            }
        }

    Returns:
        (Response): 标准响应的json数据包含code、msg和data字段，如果异常则code不为0，下同
    """
    return make_json_response(data=settings.common_settings.SKU_LIMIT.dict())


@custom_hmi_bp.route("/example/sku_limit", methods=["POST"])
@req_log(log=wcs_log)
@validator()
def update_sku_limit(body: SkuLimit):
    """演示通过HMI【辅助功能】界面发送请求到本接口, 接收HMI更新SKU的长宽高重的大小限制，需要注意数值单位
    本接口的路径是: api/custom/example/update_sku_limit

    Returns:
        (Response): 标准响应的json数据包含code、msg和data字段，如果异常则code不为0，下同
    """
    # data = request.get_json()
    # validate_data = (**data)
    # TODO: 可以按需增加数据格式校验
    settings.common_settings.SKU_LIMIT = body
    # 保存通用配置的改动
    settings.common_settings.dumps()
    return make_json_response(msg="数据更新成功！")


@custom_hmi_bp.route("/example/update_task_done_number", methods=["POST"])
@req_log(log=wcs_log)
def example_update_task_done_number():
    """演示通过HMI【辅助功能】界面发送请求到本接口, 接收HMI更新当前任务的已完成数量的数据，
    本接口的路径是: api/custom/example/update_task_done_number

    Returns:
        (Response): 标准响应的json数据包含code、msg和data字段，如果异常则code不为0，下同
    """
    data = request.get_json()
    wcs_log.info(f"【辅助功能示例接口 update_task_done_number】接收到更新任务数量数据: {data}")
    return make_json_response(msg="服务端已成功接收到请求数据，但由于是示例接口，数据不会生效。")


@custom_hmi_bp.route("/example/single_class_pal_task", methods=["POST"])
@req_log(log=wcs_log)
@validator()
def example_single_class_pal_task(body: SingleTaskCreateSchema):
    """演示示例, 本接口地址: api/custom/example/single_class_pal_task
    """
    wcs_log.info(f"【辅助功能示例接口 single_class_pal_task】接收到单码任务数据: {body.json()}")
    # TODO: 这里可以实现处理接收请求的相关逻辑
    return make_json_response(msg="服务端已成功接收到请求数据，但由于是示例接口，数据不会生效。")


@custom_hmi_bp.route("/example/pallet_is_clear", methods=["POST"])
@req_log(log=wcs_log)
def example_pallet_is_clear():
    """演示示例, 本接口地址: api/custom/example/pallet_is_clear
    """
    data = request.get_json()
    wcs_log.info(f"【辅助功能示例接口 pallet_is_clear】接收到数据: {data}")
    if "pallet_id" not in data:
        return make_json_response(code=-1, msg="请求参数错误: 缺少pallet_id字段")
    # TODO: 这里可以实现处理接收请求的相关逻辑
    return make_json_response(msg="服务端已成功接收到请求数据，但由于是示例接口，数据不会生效。")


@custom_hmi_bp.route("/example/piece_picking_order", methods=["POST"])
@req_log(log=wcs_log)
def example_piece_picking_order():
    """演示示例, 本接口地址: api/custom/example/piece_picking_order
    """
    data = request.get_json()
    wcs_log.info(f"【辅助功能示例接口 piece_picking_order】接收到分拣站订单数据: {data}")
    # TODO: 这里可以实现处理接收请求的相关逻辑
    return make_json_response(msg="服务端已成功接收到请求数据，但由于是示例接口，数据不会生效。")



