# -*- coding: utf-8 -*-
"""
Copyright (c) XYZ Robotics Inc. - All Rights Reserved
Unauthorized copying of this file, via any medium is strictly prohibited
Proprietary and confidential
Author: Yuhang Wu <yuhang.wu@xyzrobotics.ai>, 2022-07-04

存放与历史订单表相关的CRUD方法.
"""
from typing import Iterable, List, Optional

from sqlalchemy.orm.attributes import QueryableAttribute

from apps import db
from apps.exceptions import XYZNotFoundError
from apps.schemas import QuerySchema
from wcs_adaptor.models import HistoryTaskModel as TaskHistoryDBModel
from .entity import TaskHistory as TaskHistoryEntity


class CURDTaskHistory(object):

    def all(self) -> List[TaskHistoryEntity]:
        """返回所有历史任务."""
        queryset = db.session.query(TaskHistoryDBModel).filter()
        data = TaskHistoryDBModel.to_entities(queryset)
        return data

    def search(self, qs: QuerySchema) -> List[TaskHistoryEntity]:
        """返回一组历史任务实体对象."""
        filters = qs.to_sa_filters()
        collation = qs.to_sa_collation()
        queryset = db.session.query(TaskHistoryDBModel) \
            .filter(*filters) \
            .order_by(*collation) \
            .limit(qs.page_size) \
            .offset((qs.page - 1) * qs.page_size)

        ret = queryset.first()
        if ret is None:
            # 未找到记录
            data = []
        else:
            data = TaskHistoryDBModel.to_entities(queryset)
        return data

    def count(self, filters: Iterable[QueryableAttribute]) -> int:
        """根据条件统计数据量."""
        count = db.session.query(TaskHistoryDBModel).filter(*filters).count()
        return count

    def get_task(self, pk: int) -> Optional[TaskHistoryEntity]:
        """根据主键ID获取任务实体对象."""
        task: Optional[TaskHistoryDBModel] = db.session.query(
            TaskHistoryDBModel
        ).filter(
            TaskHistoryDBModel.id == pk
        ).first()
        if task:
            return task.entity

    def get_task_or_404(self, pk: int) -> TaskHistoryEntity:
        """根据主键获取任务实体对象, 如果不存在将抛出异常.

        Raises:
            XYZNotFoundError: 记录不存在时抛出异常.

        """
        task: Optional[TaskHistoryDBModel] = db.session.query(
            TaskHistoryDBModel
        ).filter(
            TaskHistoryDBModel.id == pk
        ).first()
        if task is None:
            raise XYZNotFoundError(error_message="未找到此记录")
        return task.entity

    def delete_all(self):
        """删除所有历史任务."""
        db.session.query(TaskHistoryDBModel).filter().delete()
        db.session.commit()

    def last(self) -> Optional[TaskHistoryEntity]:
        """返回最近添加的一条记录."""
        result: Optional[TaskHistoryDBModel] = db.session.query(
            TaskHistoryDBModel
        ).order_by(TaskHistoryDBModel.id.desc()).first()  # type: ignore
        if result:
            return result.entity


crud_task_history = CURDTaskHistory()
