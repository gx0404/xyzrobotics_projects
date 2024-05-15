#  -*- coding: utf-8 -*-
#  Copyright (c) XYZ Robotics Inc. - All Rights Reserved
#  Unauthorized copying of this file, via any medium is strictly prohibited
#  Proprietary and confidential
#  Author: Yuhang Wu <yuhang.wu@xyzrobotics.ai>, 2022/10/24 上午10:22
from .crud import crud_planning_result
from .schemas import Response


def get_planning_result_by_order_id(order_id: str) -> Response:
    """根据订单ID获取离线规划结果.

    必须在 app.app_context() 上下文环境内调用.

    Args:
        order_id(str): 订单编号

    Returns:
        SingleResponseSchema:
            {
                "code": 0,
                "msg": "success",
                "data": {
                    id: int
                    order_id: str
                    status: PlanningStatus
                    result: Optional[PlanningResult] = None
                    start_time: Optional[datetime] = None
                    end_time: Optional[datetime] = None
                    create_time: Optional[datetime] = None
                }
            }
    """
    record = crud_planning_result.get_or_404_by_order_id(order_id)
    return Response(data=record)
