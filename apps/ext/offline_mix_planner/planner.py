#  -*- coding: utf-8 -*-
#  Copyright (c) XYZ Robotics Inc. - All Rights Reserved
#  Unauthorized copying of this file, via any medium is strictly prohibited
#  Proprietary and confidential
#  Author: Yuhang Wu <yuhang.wu@xyzrobotics.ai>, 2022/8/31 下午1:30
import os
import logging
import typing
import multiprocessing
from datetime import datetime
from multiprocessing import Pool
from concurrent.futures.thread import ThreadPoolExecutor
from threading import Event
from typing import Union, List

from apps.exceptions import XYZBaseError, XYZIntegrityError
from apps.models import start_transaction
from apps.settings import settings
from .crud import crud_planning_result
from .entity import PlanningResult, PlanningResultRecord
from .logger import logger
from .schemas import CreateSchema, UpdateSchema
from .signals import plan_finished, plan_failure
from .enums import PlanningStatus

if typing.TYPE_CHECKING:
    from wcs_adaptor.entity import SkuInfo
    from multiprocessing.pool import ApplyResult


def f(_):
    process = multiprocessing.current_process()
    return process.pid


def get_planning_result(
    order_id: str,
    mpp,
    sp,
    box_type,
    sku_tool_map,
    sku_info: List["SkuInfo"],
    max_pallet_num: int,
    conversion_dict: dict,
    to_ws: str,
    place_corner_id: int = 0,
    robot_id: str = "0",
) -> typing.Optional[PlanningResult]:
    """获取规划结果.

    Args:
        order_id:
        mpp:
        sp:
        sku_info:
        max_pallet_num:
        to_ws:
        place_corner_id:
        robot_id:

    Returns:
        PlanningResult: 规划结果对象
    """
    try:
        from xyz_bin_packing.offline_post_validation import OfflineMixedPlanner
    except ImportError as e:
        message = (
            "导入 xyz-bin-packing 失败，可能有如下原因：\n"
            "1. 未安装 xyz-bin-packing;\n"
            "2. xyz-bin-packing 的版本过低，请升级到 2.3.5 以上版本;\n"
            "3. 未激活运行环境，请使用命令 `source /opt/xyz/apollo_setup.bash` 激活环境，或使用 3.0.14 以上版本的 central_hub_gui 选择【混码项目】预设节点."
        )
        raise ImportError(message) from e

    sku_info_map = {sku.sku_id: sku for sku in sku_info}

    try:
        planner = OfflineMixedPlanner(
            sku_info_map=sku_info_map,
            max_pallet_num=max_pallet_num,
            conversion_dict=conversion_dict,
            order_id=order_id,
            mpp=mpp,
            sp=sp,
            box_type=box_type,
            sku_tool_map=sku_tool_map,
            place_workspace_id=to_ws,
            place_corner_id=place_corner_id,
            robot_id=robot_id,
            log_level="DEBUG" if settings.DEBUG else logging.ERROR,
        )
        plan_data: list
        pallet_num: int
        chosen_comb: list
        sku_pallet: dict
        (
            outcome,
            plan_data,
            chosen_comb,
            pallet_num,
            sku_pallet,
        ) = planner.offline_planning()
    except Exception as e:
        logger.error(f"Offline planning failed: {e}", exc_info=True)
        raise e
    if outcome:
        try:
            print("plan_data", plan_data)
            return PlanningResult.parse_obj(
                {
                    "batch_results": plan_data,
                    "chosen_comb": chosen_comb,
                    "pallet_num": pallet_num,
                    "sku_pallet": sku_pallet,
                }
            )
        except Exception as e:
            logger.error(f"Offline planning result parse failed: {e}", exc_info=True)
            raise e


def get_planning_result_process(
    sku_info: List["SkuInfo"],
    max_pallet_num: int,
    conversion_dict: dict,
    to_ws: str,
    order_id: str,
    mpp,
    sp,
    box_type,
    sku_tool_map,
    place_corner_id: int = 0,
    robot_id: str = "0",
) -> Union[PlanningResult, XYZBaseError, None]:
    """获取规划结果的进程函数.

    Args:
        sku_info: sku信息
        max_pallet_num: 最大托盘数量
        to_ws: 目标工作站
        place_corner_id: 放置角落id
        robot_id: 机器人id
        order_id: 订单号

    Returns:
        PlanningResult or XYZBaseError: 规划成功返回规划结果, 否则返回错误信息
    """
    try:
        results = get_planning_result(
            order_id=order_id,
            mpp=mpp,
            sp=sp,
            box_type=box_type,
            sku_tool_map=sku_tool_map,
            sku_info=sku_info,
            max_pallet_num=max_pallet_num,
            conversion_dict=conversion_dict,
            to_ws=to_ws,
            place_corner_id=place_corner_id,
            robot_id=robot_id,
        )
        logger.info(f"Offline planning result: {results}")
        return results
    except Exception as e:
        error = XYZBaseError(error_message="Offline planning failed")
        logger.error(f"Offline planning failed: {e}", exc_info=True)
        return error


class OfflineMixedTaskPlanner:
    """离线混码任务规划器.

    这是一个单例类，用于离线混码任务规划.
    单例模式且 `__init__` 只调用一次
    """

    _instances = None
    _init = None

    def __new__(cls, *args, **kwargs):
        if OfflineMixedTaskPlanner._instances is None:
            OfflineMixedTaskPlanner._instances = super().__new__(cls, *args, **kwargs)
        return OfflineMixedTaskPlanner._instances

    def __init__(self):
        if self._init:
            return
        self._init = 1
        assert (
            settings is not None
            and settings.dpt_settings is not None
            and settings.dpt_settings.offline_mix_planner is not None
        )
        self.timeout = settings.dpt_settings.offline_mix_planner.timeout
        self.max_workers = settings.dpt_settings.offline_mix_planner.max_workers
        # 上一个规划任务完成事件
        self.__last_task_is_done_event = Event()

        # 执行规划的工作进程池
        self.__plan_workers = Pool(processes=self.max_workers)

        # 用于等待规划结果的线程池
        self.__result_monitor = ThreadPoolExecutor(max_workers=self.max_workers)

        self.__pids = self.__plan_workers.map(f, range(self.max_workers))

    def submit(
        self,
        sku_info: List["SkuInfo"],
        max_pallet_num: int,
        conversion_dict: dict,
        to_ws: str,
        order_id: str,
        mpp,
        sp,
        box_type,
        sku_tool_map,
        place_corner_id: int = 0,
        robot_id: str = "0",
        success_callback: typing.Optional[typing.Callable] = None,
        failure_callback: typing.Optional[typing.Callable] = None,
    ):
        """开启一个离线混码规划任务.

        将会在一个新的进程中执行规划任务，由线程池中的线程监控进程的执行结果，根据执行结果调用相应的回调函数.
            - 规划完成，会将规划结果发送给`plan_finished`信号;
            - 规划失败，会将错误信息发送给`plan_failure`信号.

        该任务为异步任务，在规划完成后会根据 `task_cls` 创建任务，并更新到 `order` 中.

        Args:
            sku_info: sku信息
            max_pallet_num: 最大托盘数量
            to_ws: 目标工作站
            place_corner_id: 放置角落id
            robot_id: 机器人id
            order_id: 订单号
            success_callback: 规划成功后的回调函数
            failure_callback: 规划失败后的回调函数

        Raises:
            XYZBaseError: 规划失败时将抛出错误
        """
        if self.__last_task_is_done_event.is_set():
            raise XYZBaseError("上一个规划任务正在规划中, 请稍后...")

        try:
            # 在数据表中新建规划记录
            with start_transaction() as session:
                planning_record = crud_planning_result.create(
                    session=session, create=CreateSchema(order_id=order_id)
                )
                self.__last_task_is_done_event.set()

            # WARN: 异步规划目前没有开始时间, 开始时间是在子进程开始执行时记录
            #  由于进程间中不能共享数据库会话, 所以 `start_time` 目前为空.

            result = self.__plan_workers.apply_async(
                get_planning_result_process,
                args=(
                    sku_info,
                    max_pallet_num,
                    conversion_dict,
                    to_ws,
                    order_id,
                    mpp,
                    sp,
                    box_type,
                    sku_tool_map,
                    place_corner_id,
                    robot_id,
                ),
            )
            self.__result_monitor.submit(
                self._monitor_result,
                planning_record,
                result,
                success_callback,
                failure_callback,
            )
        except XYZIntegrityError as err:
            e = XYZBaseError(error_message="离线混码规划任务提交失败，原因：订单号已存在")
            plan_failure.send(
                callback=failure_callback,
                error=e,
            )
            raise e from err
        except Exception as exc:
            plan_failure.send(
                callback=failure_callback,
                error=XYZBaseError(error_message="离线混码规划任务提交失败，原因：{}".format(exc)),
            )
            raise XYZBaseError("Offline planning failed") from exc

    def await_async_result(self, result: "ApplyResult") -> PlanningResult:
        """获取规划结果

        Args:
            result(AsyncResult): 进程池的异步结果对象

        Returns:
            PlanningResult

        Raises:
            XYZBaseError: 规划返回了, 但结果不正确
            multiprocessing.TimeoutError: 规划超时
            Exception: 未知异常
        """
        error = None
        try:
            # 由于是异步任务，这里需要等待任务完成
            results = result.get(timeout=self.timeout)
            if isinstance(results, PlanningResult):
                return results
            elif isinstance(results, XYZBaseError):
                error = results.error_message
            elif results is None:
                error = "No plan result."
            else:
                error = f"Unknown plan result type: {type(results)}"
            raise XYZBaseError(error_message=error)
        except multiprocessing.TimeoutError as err:
            # HACK: 进程池因子模块被阻塞, 强制结束子进程, 并重新创建进程池
            #  当前这种方式过于暴力 🐵
            #  在退出后端程序时, 会因为丢失文件描述符而报错(), 可以忽略
            #  OSError: [Errono 9] Bad file descriptor

            for pid in self.__pids:
                os.system("kill -9 {}".format(pid))

            self.__plan_workers = Pool(processes=self.max_workers)
            self.__pids = self.__plan_workers.map(f, range(self.max_workers))
            raise err
        finally:
            self.__last_task_is_done_event.clear()

    def _monitor_result(
        self,
        planning_record: PlanningResultRecord,
        result: "ApplyResult",
        success_callback: typing.Optional[typing.Callable] = None,
        failure_callback: typing.Optional[typing.Callable] = None,
    ):
        """监控规划结果."""
        success_callback = success_callback or self._default_success_callback
        failure_callback = failure_callback or self._default_failure_callback

        try:
            planning_record.result = self.await_async_result(result)
            plan_finished.send(callback=success_callback, record=planning_record)
        except multiprocessing.TimeoutError:
            error = f"订单({planning_record.order_id})的离线混码规划任务超时({self.timeout}s)"
            logger.error(error, exc_info=True)
            plan_failure.send(
                callback=failure_callback,
                error=XYZBaseError(error_message=error),
                record=planning_record,
            )
        except Exception as e:
            logger.error("离线混码规划失败", exc_info=True)
            plan_failure.send(
                callback=failure_callback,
                error=XYZBaseError(error_message=str(e)),
                record=planning_record,
            )

    def plan(
        self,
        sku_info: List["SkuInfo"],
        max_pallet_num: int,
        conversion_dict: dict,
        to_ws: str,
        order_id: str,
        mpp,
        sp,
        box_type,
        sku_tool_map,
        place_corner_id: int = 0,
        robot_id: str = "0",
    ) -> typing.Optional[PlanningResult]:
        """同步执行离线混码规划任务.

        调用该方法将会阻塞当前线程，直到规划完成.

        Args:
            sku_info: sku信息
            max_pallet_num: 最大托盘数量
            to_ws: 目标工作站
            place_corner_id: 放置角落id
            robot_id: 机器人id
            order_id: 订单号

        Returns:
            Results or None: 规划结果, 规划失败时返回None
        """
        if self.__last_task_is_done_event.is_set():
            raise XYZBaseError("上一个规划任务正在规划中, 请稍后...")

        with start_transaction() as session:
            planning_record = crud_planning_result.create(
                session=session, create=CreateSchema(order_id=order_id)
            )
            self.__last_task_is_done_event.set()
            try:
                crud_planning_result.start_planning(
                    order_id=planning_record.order_id,
                    update=UpdateSchema(start_time=datetime.now()),
                    session=session,
                )

                async_result = self.__plan_workers.apply_async(
                    get_planning_result,
                    kwds=dict(
                        order_id=order_id,
                        mpp=mpp,
                        sp=sp,
                        box_type=box_type,
                        sku_tool_map=sku_tool_map,
                        sku_info=sku_info,
                        max_pallet_num=max_pallet_num,
                        conversion_dict=conversion_dict,
                        to_ws=to_ws,
                        place_corner_id=place_corner_id,
                        robot_id=robot_id,
                    ),
                )

                results = self.await_async_result(
                    result=async_result,
                )

                crud_planning_result.patch(
                    session=session,
                    pk=planning_record.id,
                    update=UpdateSchema(
                        status=PlanningStatus.FINISHED,
                        result=results,
                        end_time=datetime.now(),
                    ),
                )
            except multiprocessing.TimeoutError as e:
                error = f"订单({planning_record.order_id})的离线混码规划任务超时({self.timeout}s)"
                crud_planning_result.patch(
                    session=session,
                    pk=planning_record.id,
                    update=UpdateSchema(
                        status=PlanningStatus.FAIL,
                        end_time=datetime.now(),
                        is_deleted=None,
                    ),
                )
                raise XYZBaseError(error_message=error) from e
            except Exception as e:
                crud_planning_result.patch(
                    session=session,
                    pk=planning_record.id,
                    update=UpdateSchema(
                        status=PlanningStatus.FAIL,
                        end_time=datetime.now(),
                        is_deleted=None,
                    ),
                )
                raise XYZBaseError(error_message=f"离线混码规划失败") from e
            else:
                return results
            finally:
                # WARN: 这里必须显式的调用 commit 不然数据不会生效
                session.commit()

    @staticmethod
    def _default_success_callback(record: PlanningResultRecord, **kwargs):
        """默认的规划成功回调函数."""
        logger.info("规划成功")

    @staticmethod
    def _default_failure_callback(error, **kwargs):
        """默认的规划失败回调函数."""
        logger.error("规划失败")

    def shutdown(self):
        """终止规划."""
        logger.info("正在关闭离线混码规划进程...")
        self.__plan_workers.terminate()
        self.__result_monitor.shutdown()
        logger.info("已关闭离线混码规划进程")
