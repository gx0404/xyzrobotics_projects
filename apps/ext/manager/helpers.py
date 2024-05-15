#  -*- coding: utf-8 -*-
#  Copyright (c) XYZ Robotics Inc. - All Rights Reserved
#  Unauthorized copying of this file, via any medium is strictly prohibited
#  Proprietary and confidential
#  Author: Yuhang Wu <yuhang.wu@xyzrobotics.ai>, 2022/9/8 下午3:14
import pickle
from functools import wraps
from typing import Callable

from apps.models import start_transaction
from apps.log import wcs_log, outside_log


# TODO(YuhangWu): 选择 MongoDB 作为 Manager 的存储引擎，则以下注销以下三个函数的功能。
class ManagerHelper(object):
    @staticmethod
    def load_task():
        """加载存储的任务备份"""
        from wcs_adaptor.manager import order_manager, task_manager, workspace_manager
        from wcs_adaptor.models import (
            TaskManagerModel,
            WorkspaceManagerModel,
            OrderManagerModel,
            DeprecatedTaskManagerModel,
            TaskManager,
        )
        from wcs_adaptor.exceptions import WCSError

        if model := DeprecatedTaskManagerModel.query.first():
            old_task_manager: TaskManager = pickle.loads(model.task_manager_data)
            old_order_manager: TaskManager = pickle.loads(
                model.offline_task_manager_data
            )
            if old_task_manager.task_list or old_order_manager.task_list:
                raise WCSError("管理器加载失败, 当前版本不兼容, 请先使用旧版本完成剩余任务!")

        if model := TaskManagerModel.query.first():
            outside_log.info(f"Load task, backup time:{model.backup_time}")
            task_manager.loads(model.bin)
        else:
            outside_log.info("No saved task")
            return {"error": 1, "error_message": "No saved task"}

        if model := WorkspaceManagerModel.query.first():
            outside_log.info(f"Load workspace, backup time:{model.backup_time}")
            workspace_manager.loads(model.bin)
        else:
            outside_log.info("no workspace.")
            return {"error": 1, "error_message": "No saved task"}

        if model := OrderManagerModel.query.first():
            outside_log.info(f"Load order, backup time:{model.backup_time}")
            order_manager.loads(model.bin)
        else:
            outside_log.info("no workspace.")
            return {"error": 1, "error_message": "No saved task"}

        return {"error": 0, "code": 0, "msg": "", "error_message": "", "data": None}

    @staticmethod
    def backup_manager():
        """备份manager."""
        from wcs_adaptor.manager import order_manager, task_manager, workspace_manager
        from wcs_adaptor.models import (
            TaskManagerModel,
            WorkspaceManagerModel,
            OrderManagerModel,
        )

        with start_transaction() as session:
            def _dumps(model, manager):
                ret = session.query(model).first()
                if not ret:
                    ret = model()
                    session.add(ret)
                ret.bin = manager.dumps()

            # 持久化订单管理器
            _dumps(OrderManagerModel, order_manager)
            # 持久化任务管理器
            _dumps(TaskManagerModel, task_manager)
            # 持久化工作空间
            _dumps(WorkspaceManagerModel, workspace_manager)

        #wcs_log.info("backup success!")

    @staticmethod
    def backup_manager_wrapper() -> Callable:
        """备份manager的装饰器.

        Examples:
            @backup_manager_wrapper()
            def test():
                pass

        """

        def decorate(func: Callable) -> Callable:
            @wraps(func)
            def wrapper(*args, **kwargs):
                from apps import cached_app
                app = cached_app()
                with app.app_context():
                    ret = func(*args, **kwargs)
                    ManagerHelper.backup_manager()
                    return ret

            return wrapper

        return decorate
