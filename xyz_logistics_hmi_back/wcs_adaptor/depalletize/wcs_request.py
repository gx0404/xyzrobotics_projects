# -*- coding: utf-8 -*-
import time
import typing
from typing import Optional, Tuple, Union

import requests
from requests.exceptions import RequestException

from apps import _, create_app, json_dumper, mp, settings, wcs_log
from wcs_adaptor.depalletize.entity import DPTTask as Task
from wcs_adaptor.entity import LiteOrder as Order
from wcs_adaptor.helpers import backup_manager_wrapper
from wcs_adaptor.manager import order_manager, workspace_manager
from wcs_adaptor.zh_msg import REPORT_EXCEPTION_CODES

if typing.TYPE_CHECKING:
    from requests import Response

    from apps.exceptions import XYZBaseError
    from apps.ext.offline_mix_planner.entity import PlanningResult

NOTICE_SYSTEM_IS_READY_API = "request/wcs/notice_system_is_ready"
NOTICE_PICK_WS_IS_EMPTY_API = "request/wcs/notice_pick_ws_is_empty"
GET_SKU_INFO_FROM_WCS = "request/wcs/get_sku_info"
NOTICE_PALLET_IS_FULL_API = "request/wcs/multi_class_pal_task/note_pallet_is_full"
REPORT_PLAN_RESULT_API = "request/wcs/multi_class_pal_task/report_plan_result"

"""基础接口"""
REPORT_EXCEPTION = "request/wcs/report_exception"
TURN_AGV_API = "request/wcs/turn_agv"
REPORT_ROBOT_STATUS = "request/wcs/robot_status"
INIT_ERROR_API = "request/wcs/init_error"

"""拣配任务接口"""
#拣配放置通知放满
NOTICE_DEPAL_PLACE_WS_IS_FULL_API = "request/wcs/depal_task/place_is_full"
REPORT_DEPAL_ACTION_STATUS_API = "request/wcs/depal_task/report_action_status"
REPORT_DEPAL_TASK_STATUS_API = "request/wcs/depal_task/report_task_status"

"""合托任务接口"""
REPORT_MERGE_TASK_STATUS_API = "request/wcs/merge_task/report_task_status"

"""输送线空箱回收任务接口"""
REPORT_MULTI_ACTION_STATUS_API = "/request/wcs/conveyor_pal_task/report_action_status"
REPORT_MULTI_TASK_STATUS_API = "/request/wcs/conveyor_pal_task/report_task_status"
NOTICE_MULTI_PLACE_WS_IS_FULL_API = "request/wcs/conveyor_pal_task/place_is_full"
NOTICE_PICK_COMPLETE = "request/wcs/pick_complete"
GET_CONVEYOR_TOTE_TYPE_ONLINE = "request/wcs/conveyor_pal_task/pick_ws"
"""笼车空箱回收任务接口"""
REPORT_PAL_ACTION_STATUS_API = "request/wcs/pallet_pal_task/report_action_status"
REPORT_PAL_TASK_STATUS_API = "request/wcs/pallet_pal_task/report_task_status"
NOTICE_PALLET_PLACE_WS_IS_FULL_API = "request/wcs/pallet_pal_task/place_is_full"
GET_CONVEYOR_TOTE_TYPE_PAL = "request/wcs/pallet_pal_task/pick_ws"
# 配置API地址选择接口类型
# Options:
# 單拆： "request/wcs/single_class_depal_task/report_action_status"
# 單碼： "request/wcs/single_class_pal_task/report_action_status"
# 混拆： "request/wcs/multi_class_depal_task/report_action_status"
# 混碼： "request/wcs/multi_class_pal_task/report_action_status"

# Options:
# 單拆： "request/wcs/single_class_depal_task/report_task_status"
# 單碼： "request/wcs/single_class_pal_task/report_task_status"
# 混拆： "request/wcs/multi_class_depal_task/report_task_status"
# 混碼： "request/wcs/multi_class_pal_task/report_task_status"


app = create_app()


def send_request(
    path: str,
    data: dict,
    retries: int = None,
    timeout: Union[float, Tuple[float, float]] = 3,
) -> Optional[requests.Response]:
    """发送请求到WCS.

    Args:
        path(str): 接口路径.
        data(dict): 请求体.
        retries(int): 可重试的次数, 默认一直重试.
        timeout(Union[float, Tuple[float, float]]): 默认3秒, 单次请求的超时时间
            接受一个 float 类型的数据, 表示
            也可接受一个元组, 元组内包含两个float, 分别表示, 连接超时时间 和 读取响应超时时间.

    Returns:
        Response: 响应对象.

    """
    url = f"http://{settings.WCS_ADDR}/{path}"
    is_always = retries is None
    current_retries = 0

    with requests.Session() as session:
        while is_always or current_retries < retries:
            try:
                wcs_log.info(f"[HTTP] - request - {url} - {json_dumper(data)}")
                resp = session.post(url, json=data, timeout=timeout)
                resp.raise_for_status()
                wcs_log.info(f"[HTTP] - response - {url} - {json_dumper(resp.content)}")
                return resp
            except TypeError as err:
                wcs_log.error(f"[HTTP] - error - {str(err)}")
                raise err
            except RequestException as e:
                current_retries += 1
                wcs_log.warning(
                    "Please check whether the Mockoon server is running if current environment is not production."
                )
                error_msg = (
                    f"WCS Connection Error, "
                    f"Retrying: {current_retries}/"
                    f"{'inf' if is_always else retries} "
                    f"Details:\n{str(e)}"
                )
                wcs_log.error(error_msg)
                with app.app_context():
                    mp.order.error(error_msg)
                time.sleep(0.2)


"""基础接口"""
def turn_agv(ws_id,agv_direction):
    data = {
        "ws_id":ws_id,
        "agv_direction":agv_direction,
        "agv_speed":1
    }
    return_data = send_request(TURN_AGV_API, data)
    return return_data

def report_init_error():
    data = {
        
    }
    send_request(INIT_ERROR_API, data)
    mp.order.info(f"通知wcs异常复位")
    


"""拣配任务接口"""
@backup_manager_wrapper()
def notice_depal_place_ws_is_full(place_id):
    """通知WCS托盘已满"""
    data = {
        "to_ws": place_id,
    }
    # 获取工作空间对象.
    ws = workspace_manager.get(ws_id=place_id)
    # 设置当前工作空间状态为未就绪
    ws.not_ready()
    if ws.has_pallet():
        pallet_id = ws.last_item.id
        data["pallet_id"] = pallet_id
        msg = _("已回报拣配任务放置位工作空间({0})已满, 托盘号({1})").format(place_id, pallet_id)
    else:
        msg = _("已回报拣配任务放置位工作空间({0})已满").format(place_id)
    send_request(NOTICE_DEPAL_PLACE_WS_IS_FULL_API, data)
    mp.order.info(msg)


def report_depal_action_status(task: Task, pick_num: int,from_ws,to_ws,pick_tote_data,place_tote_data):
    """回报WCS一次抓取结果"""
    data = {
        "task_id": task.task_id,
        "pick_num": pick_num,
        "action_status": 0,
        "from_ws":from_ws,
        "to_ws":to_ws,
        "pick_tote_data":pick_tote_data,
        "place_tote_data":place_tote_data,
        "message": "",
    }
    send_request(REPORT_DEPAL_ACTION_STATUS_API, data)


@backup_manager_wrapper()
def report_depal_task_finish(task: Task,pallet_tote_data,agv_direction,error):
    """回报WCS任务完成"""
    # 仅当状态为 已完成 或 已终止 才回报
    if not (task.is_finished() or task.is_terminated() or task.is_ended()):
        return
    data = {
        "task_id": task.task_id,
        "task_status": error,
        "pick_num": task.done_num,
        "target_num": task.target_num,
        "pallet_tote_data":pallet_tote_data,
        "agv_direction":agv_direction,
        "message": task.task_status.name,
        "customized_result": {},
    }
    send_request(REPORT_DEPAL_TASK_STATUS_API, data)
    mp.order.info(_("回报拣配任务{0}已完成").format(task.task_id))

"""合托任务接口"""
@backup_manager_wrapper()
def report_merge_task_finish(task: Task,pallet_tote_data,error):
    """回报WCS任务完成"""
    # 仅当状态为 已完成 或 已终止 才回报
    if not (task.is_finished() or task.is_terminated() or task.is_ended()):
        return
    data = {
        "task_id": task.task_id,
        "task_status": error,
        "pallet_tote_data":pallet_tote_data,
        "message": task.task_status.name,
        "customized_result": {},
    }
    send_request(REPORT_MERGE_TASK_STATUS_API, data)
    mp.order.info(_("回报合托任务{0}已完成").format(task.task_id))



"""输送线空箱回收任务接口"""
def report_multi_pal_action_status(task: Task, pick_num: int,to_ws):
    """回报WCS一次抓取结果"""
    data = {
        "task_id": task.task_id,
        "pick_num": pick_num,
        "action_status": 0,
        "to_ws":to_ws,
        "customized_result": {},
        "message": "",
    }
    send_request(REPORT_MULTI_ACTION_STATUS_API, data)
    mp.order.info(_("回报输送线空箱回收任务{0}单次动作,已放置到{1}空间").format(task.task_id,to_ws))

#获取输送线物料信息
def get_conveyor_tote_type_online(task: Task):
    data = {
        "task_id": task.task_id
    }
    return_data = send_request(GET_CONVEYOR_TOTE_TYPE_ONLINE, data)
    return return_data

def notice_pick_complete(task: Task):
    data = {
        "task_id": task.task_id
    }   
    return_data = send_request(NOTICE_PICK_COMPLETE, data)
    return return_data

@backup_manager_wrapper()
def report_multi_pal_task_finish(task: Task,error):
    """回报WCS任务完成"""
    # 仅当状态为 已完成 或 已终止 才回报
    if not (task.is_finished() or task.is_terminated() or task.is_ended()):
        return
    data = {
        "task_id": task.task_id,
        "task_status": error,
        "message": task.task_status.name,
        "customized_result": {},
    }
    send_request(REPORT_MULTI_TASK_STATUS_API, data)
    mp.order.info(_("回报输送线空箱回收任务{0}已完成").format(task.task_id))

@backup_manager_wrapper()
def notice_multi_pal_place_ws_is_full(place_id):
    """通知WCS托盘已满"""
    data = {
        "to_ws": place_id,
    }
    # 获取工作空间对象.
    ws = workspace_manager.get(ws_id=place_id)
    # 设置当前工作空间状态为未就绪
    ws.not_ready()
    if ws.has_pallet():
        pallet_id = ws.last_item.id
        data["pallet_id"] = pallet_id
        msg = _("已回报输送线空箱回收任务放置位工作空间({0})已满, 托盘号({1})").format(place_id, pallet_id)
    else:
        msg = _("已回报输送线空箱回收任务放置位工作空间({0})已满").format(place_id)
    send_request(NOTICE_MULTI_PLACE_WS_IS_FULL_API, data)
    mp.order.info(msg)


"""笼车码垛任务接口"""

#笼车码垛单次动作完成
def report_pallet_action_status(task: Task, pick_num: int,to_ws):
    """回报WCS一次抓取结果"""
    data = {
        "task_id": task.task_id,
        "pick_num": pick_num,
        "action_status": 0,
        "to_ws":to_ws,
        "customized_result": {},
        "message": "",
    }
    send_request(REPORT_PAL_ACTION_STATUS_API, data)

#笼车码垛任务完成
@backup_manager_wrapper()
def report_pallet_task_finish(task: Task,error):
    """回报WCS任务完成"""
    # 仅当状态为 已完成 或 已终止 才回报
    if not (task.is_finished() or task.is_terminated() or task.is_ended()):
        return
    data = {
        "task_id": task.task_id,
        "task_status": error,
        "pick_num":task.done_num,
        "message": task.task_status.name,
        "customized_result": {},
    }
    send_request(REPORT_PAL_TASK_STATUS_API, data)
    mp.order.info(_("笼车空箱回收任务{0}已完成").format(task.task_id))

#获取输送线物料信息
def get_conveyor_tote_type_pal(task: Task):
    data = {
        "task_id": task.task_id
    }
    return_data = send_request(GET_CONVEYOR_TOTE_TYPE_PAL, data,timeout=20)
    return return_data

@backup_manager_wrapper()
def notice_pallet_pal_place_ws_is_full(place_id):
    """通知WCS托盘已满"""
    data = {
        "to_ws": place_id,
    }
    # 获取工作空间对象.
    ws = workspace_manager.get(ws_id=place_id)
    # 设置当前工作空间状态为未就绪
    ws.not_ready()
    if ws.has_pallet():
        pallet_id = ws.last_item.id
        data["pallet_id"] = pallet_id
        msg = _("已回报输送线空箱回收任务放置位工作空间({0})已满, 托盘号({1})").format(place_id, pallet_id)
    else:
        msg = _("已回报笼车空箱回收任务放置位工作空间({0})已满").format(place_id)
    send_request(NOTICE_PALLET_PLACE_WS_IS_FULL_API, data)
    mp.order.info(msg)


# ---------
#  基础功能
# ---------


def notice_system_is_ready():
    """通知WCS准备就绪"""
    api = NOTICE_SYSTEM_IS_READY_API
    data = {"status": 0, "message": "ready"}
    send_request(api, data)




def notice_pick_ws_is_empty(task: Task):
    """通知WCS托盘已空"""
    data = {
        "task_id": task.task_id,
        "ws_id": task.from_ws,
    }
    # 获取工作空间对象.
    ws = workspace_manager.get(ws_id=task.from_ws)
    ws.not_ready()
    if ws.has_pallet():
        pallet_id = ws.last_item.id
        data["pallet_id"] = pallet_id
        msg = _("已回报抓取位工作空间({0})已空, 托盘号({1})").format(task.from_ws, pallet_id)
    else:
        msg = _("已回报抓取位工作空间({0})已空").format(task.from_ws)
    send_request(NOTICE_PICK_WS_IS_EMPTY_API, data)
    mp.order.info(msg)


def get_sku_info_from_wcs(sku_id, customized_request):
    """XYZ 向 WCS请求SKU 信息 (扫码专用)"""

    api = GET_SKU_INFO_FROM_WCS
    data = {"sku_id": sku_id, "customized_request": customized_request}
    return send_request(api, data)


def report_exception(error_data):
    """向wcs回報異常

    Args:
        error_data:
            {
                "code": "xxx",
                "error_msg": error["error_msg"],
                "msg": msg,
                "tip": tip,
                "msg_type": "error",  # 异常等级
                "class": "motion",  # 异常来源
            }
    """
    # import ipdb;ipdb.set_trace()
    # if error_data["code"] in REPORT_EXCEPTION_CODES:
    # NOTICE: 自行构造需要发给 WCS 的数据格式
    data = {
        "task_id":error_data["task_id"],
        "error_code": error_data["code"],
        "error_msg": error_data["error_msg"],
        "msg": error_data["msg"],
    }
    resp = send_request(REPORT_EXCEPTION, data)
    mp.order.info(_("异常信息({0})已回报至WCS.".format(error_data["code"])))
    return resp


# ------------------------------------------
#  单拆接口, 单码接口， 混拆接口， 混拆online接口
# ------------------------------------------





# ---------------------------
#  混码 offline接口
# ---------------------------
def report_plan_result(
    order: Order,
    result: Optional["PlanningResult"] = None,
    error: Optional["XYZBaseError"] = None,
) -> "Response":
    """回报规划结果或异常.

    Args:
        order (Order): 订单对象
        result (PlanningResult): 规划结果对象
        error (XYZBaseError): 异常对象

    Returns:
        Response: 响应对象
    """
    # 根据实际情况构造回报数据
    if error:
        data = {
            "error_message": error.error_message,
        }
    else:
        data = {
            "order_id": order.order_id,
            "plan_result": result and result.dict(),
        }
    return send_request(REPORT_PLAN_RESULT_API, data)


@backup_manager_wrapper()
def report_action_status_offline(task: Task, pick_num: int):
    """回报WCS一次混码抓取结果"""
    # task.task_id = "xxx_y", xxx是订单ID, y是任务序号
    order_id, round_id = task.task_id.rsplit("_", 1)
    order = order_manager.get_order_or_404(order_id)
    data = {
        "task_id": task.task_id,
        "pick_num": pick_num,
        "action_status": 0,
        "round_id": round_id,
        "customized_result": {},
        "message": "",
    }
    send_request(REPORT_ACTION_STATUS_API, data)
    wcs_log.info(f"Current progress: {order.done_num}/{order.total_num}")


def report_task_finish_offline(task: Task):
    """回报WCS混码任务完成"""
    # 仅当状态为 已完成 或 已终止 才回报
    if not (task.is_finished() or task.is_terminated() or task.is_ended()):
        return
    order_id, round_id = task.task_id.rsplit("_", 1)
    order = order_manager.get_order_or_404(order_id)
    data = {
        "task_id": task.task_id,
        "order_id": order.order_id,
        "task_status": task.task_status.value,
        "order_status": order.order_status.value,
        "data": [task.json() for task in order.tasks],
        "customized_result": {},
        "message": "",
    }
    send_request(REPORT_TASK_STATUS_API, data)
    mp.order.info(_("回报任务{0}已完成").format(task.task_id))


def report_order_finish_offline(order: Order):
    """回报WCS混码订单完成"""
    # 仅当状态为 已完成 或 已终止 才回报
    if not (order.is_finished() or order.is_terminated() or order.is_ended()):
        return
    wcs_log.info("report wcs offline order finish")

    data = {
        "order_id": order.order_id,
        "order_status": order.order_status.value,
        "data": [task.json() for task in order.tasks],
        "customized_result": {},
        "message": "",
    }
    send_request(REPORT_ORDER_STATUS_API, data)
    mp.order.info(_("回报订单{0}已完成").format(order.order_id))


def multi_class_notice_place_ws_is_full(task: Task):
    """通知WCS当前放置托盘已满."""
    api = NOTICE_PALLET_IS_FULL_API
    order_id, current_index = task.task_id.rsplit("_", 1)
    order = order_manager.get_order_or_404(order_id)
    ws = workspace_manager.get(ws_id=task.to_ws)
    # 设置当前工作空间不可用
    ws.not_ready()
    data = {
        "task_id": task.task_id,
        "data": [task.json() for task in order.tasks][: int(current_index)],
    }
    # 获取同workspace_id相同的托盘ID.
    pallet = ws.get_item(task.to_ws)
    if pallet:
        data["pallet_id"] = pallet.pallet_id
    send_request(api, data)
