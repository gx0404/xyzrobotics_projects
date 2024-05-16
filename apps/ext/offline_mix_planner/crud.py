#  -*- coding: utf-8 -*-
#  Copyright (c) XYZ Robotics Inc. - All Rights Reserved
#  Unauthorized copying of this file, via any medium is strictly prohibited
#  Proprietary and confidential
#  Author: Yuhang Wu <yuhang.wu@xyzrobotics.ai>, 2022/10/21 下午3:50
from typing import Optional

from sqlalchemy.orm import Query
from sqlalchemy.orm.scoping import ScopedSession

from apps.models import db
from apps.base_app.crud import CRUDBase
from apps.exceptions import XYZNotFoundError
from .model import PlanningResultModel
from .entity import PlanningResultRecord, PlanningResult
from .enums import PlanningStatus
from .schemas import CreateSchema, UpdateSchema


class CRUDPlanningResult(
    CRUDBase[
        PlanningResultModel,
        PlanningResultRecord,
        None,
        CreateSchema,
        UpdateSchema
    ]
):

    def _get_query_statement_by_order_id(self, order_id: str, session: ScopedSession = None) -> Query:
        if not session:
            session = db.session
        return session.query(self.model).filter(self.model.order_id == order_id, self.model.is_deleted == False)

    def get_by_order_id(self, order_id: str) -> Optional[PlanningResultRecord]:
        """通过订单ID获取规划结果

        Args:
            order_id: 订单ID

        Returns:
            PlanningResult or None: 如果存在返回规划结果, 否则返回 None
        """
        queryset: Query = db.session.query(self.model)
        if result := queryset.filter(self.model.order_id == order_id, self.model.is_deleted == False).first():
            return self.entity.from_orm(result)

    def get_or_404_by_order_id(self, order_id: str) -> PlanningResultRecord:
        """通过订单ID获取规划结果, 如果不存在则抛出异常.

        Args:
            order_id: 订单ID

        Returns:
            PlanningResult or None: 如果存在返回规划结果, 否则返回 None
        """
        if result := self.get_by_order_id(order_id):
            return result
        else:
            raise XYZNotFoundError(f"订单({order_id})的规划结果不存在.")

    def is_finish(self, order_id: str) -> bool:
        """是否规划已完成."""
        result = self.get_or_404_by_order_id(order_id)
        return result.status == PlanningStatus.FINISHED

    def is_exists(self, order_id: str) -> bool:
        """通过订单ID判断规划结果是否存在"""
        queryset: Query = db.session.query(self.model)
        if queryset.filter(self.model.order_id == order_id, self.model.is_deleted == False).exists():
            return True
        return False

    def _set_status(self, order_id: str, status: PlanningStatus, session: ScopedSession):
        """设置规划状态."""
        query = self._get_query_statement_by_order_id(order_id)
        query.update({self.model.status: status})

    def set_finish_status(self, order_id: str, session: ScopedSession):
        """设置规划状态为规划完成."""
        self._set_status(order_id, PlanningStatus.FINISHED, session)

    def set_fail_status(self, order_id: str, session: ScopedSession):
        """设置规划状态为规划失败."""
        self._set_status(order_id, PlanningStatus.FAIL, session)

    def set_planning_status(self, order_id: str, session: ScopedSession):
        """设置规划状态为正在规划中."""
        self._set_status(order_id, PlanningStatus.PLANNING, session)

    def start_planning(self, order_id: str, update: UpdateSchema, session: Optional[ScopedSession] = None) -> bool:
        """开始一个规划.

        Args:
            order_id: 订单编号
            update: 更新模式对象
            session: 数据库会话对象

        Returns:
            bool: 是否执行成功
        """
        queryset: Query = session.query(self.model)
        return (
            queryset
            .filter(self.model.order_id == order_id, self.model.is_deleted == False)
            .update({
                self.model.status: PlanningStatus.PLANNING,
                self.model.start_time: update.start_time
            })
        )

    def patch(
        self,
        session: ScopedSession,
        *,
        pk: Optional[int] = None,
        update: UpdateSchema
    ) -> PlanningResultRecord:
        """更新部分字段."""
        model = self._get_raw_model(session=session, pk=pk)
        if not model:
            raise XYZNotFoundError(
                error_message=f"Patch update failed, Not Found {self.entity.__name__}({pk})."
            )
        for key, val in update.dict(exclude_unset=True).items():
            setattr(model, key, val)
        return self.entity.from_orm(model)


crud_planning_result = CRUDPlanningResult(PlanningResultModel, PlanningResultRecord)
