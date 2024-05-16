#  -*- coding: utf-8 -*-
#  Copyright (c) XYZ Robotics Inc. - All Rights Reserved
#  Unauthorized copying of this file, via any medium is strictly prohibited
#  Proprietary and confidential
#  Author: Yuhang Wu <yuhang.wu@xyzrobotics.ai>, 2022/8/31 下午2:39
from datetime import datetime
from typing import Optional

import blinker

from apps import cached_app
from apps.models import start_transaction
from .enums import PlanningStatus
from .logger import logger
from .crud import crud_planning_result
from .entity import PlanningResultRecord
from .schemas import UpdateSchema

plan_finished = blinker.signal("offline_order_planning_finished", doc="规划完成信号")
plan_failure = blinker.signal("offline_order_planning_failure", doc="规划失败信号")


@plan_finished.connect
def plan_finished_callback(sender: str, **kwargs):
    """规划完成后的回调函数."""
    with cached_app().app_context(), start_transaction() as session:
        try:
            call = kwargs.pop("callback")
            record: PlanningResultRecord = kwargs.pop("record")
            crud_planning_result.patch(
                session=session,
                pk=record.id,
                update=UpdateSchema(
                    status=PlanningStatus.FINISHED,
                    result=record.result,
                    end_time=datetime.now()
                )
            )
            call(planning_result=record.result)
            # 更新规划状态
            logger.info("混码任务规划完成，回调函数调用成功.")
        except Exception as e:
            # TODO(YuhangWu): 混码任务规划日志记录
            logger.error("混码任务规划完成, 但回调函数执行失败.", exc_info=e)


@plan_failure.connect
def plan_failure_callback(sender, **kwargs):
    """规划失败后的回调函数."""
    from apps import cached_app

    with cached_app().app_context(), start_transaction() as session:
        try:
            call = kwargs.pop("callback")
            error = kwargs.pop("error")
            record: Optional[PlanningResultRecord] = kwargs.pop("record", None)
            if record:
                # 更新状态为失败，并逻辑删除.
                crud_planning_result.patch(
                    session=session,
                    pk=record.id,
                    update=UpdateSchema(
                        status=PlanningStatus.FAIL,
                        end_time=datetime.now(),
                        is_deleted=None,
                    )
                )
            call(error=error, session=session)
            logger.info("混码任务规划失败，回调函数调用成功.")
        except Exception as e:
            logger.error("混码任务规划失败，回调函数执行失败.", exc_info=e)
