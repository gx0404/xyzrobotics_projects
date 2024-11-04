# -*- coding: utf-8 -*-
"""定义数据模型."""
import pickle
from datetime import datetime

import sqlalchemy as sa

from apps.models import db
from apps.models.base import DBModelMixin
from wcs_adaptor.enums import TaskStatus, TaskType
from wcs_adaptor.entity import LiteBaseTask, LiteOrder


# NOTICE: 已弃用，请勿使用
class DeprecatedTaskManager(object):
    """ 订单任务管理 """

    def __init__(self):
        """按照字典和列表进行存储，方便根据ID和下标进行访问
        ws_ready={
            pick_ws_id1: True
            place_ws_id1: True
            pick_ws_id2: True
            place_ws_id2: True
            ...
        }

        """
        self.task_dict = {}  # 任务字典
        self.task_list = []  # 任务列表
        self.ws_ready = {}  # 工作空间就位字典, 表示当前工作空间是否就绪
        self.clear_ws = {}  # 清除工作空间标识的字典，表示下一次是否应清空对应托盘
        self.ws_pallet_id = {}  # 可选的表示WCS发送来的当前工作空间的托盘编号
        self.terminate_task = False  # 是否要提前结束任务

    def append(self, task):
        """添加新的任务"""
        self.task_dict[task.task_id] = task
        self.task_list.append(task)

    def remove(self, task):
        """移除任务"""
        del self.task_dict[task.task_id]
        self.task_list.remove(task)

    def first(self):
        """返回第一个任务(当前任务)"""
        return self.get_task_by_index(0)

    def finish(self, task):
        """完成并移除任务"""
        self.remove(task)

    def __contains__(self, task):
        """使用in来判断是否任务已经存在"""
        return task.task_id in self.task_dict

    def __getitem__(self, i):
        """默认下标方式获取任务"""
        return self.get_task_by_index(i)

    def get_task_by_index(self, index):
        """根据下标获取任务"""
        if 0 <= index < len(self.task_list):
            return self.task_list[index]
        else:
            return None

    def get_task_by_id(self, task_id):
        """根据ID获取任务"""
        if task_id in self.task_dict:
            return self.task_dict[task_id]
        else:
            return None

    def dumps(self):
        """导出任务"""
        return pickle.dumps(self)

    def loads(self, data):
        """加载任务"""
        self.__dict__.update(pickle.loads(data).__dict__)

    def clear(self):
        """重置任务"""
        self.task_dict = {}
        self.task_list = []
        self.ws_ready = {}
        self.clear_ws = {}
        self.ws_pallet_id = {}
        self.terminate_task = False


# NOTICE: 已弃用，请勿使用
TaskManager = DeprecatedTaskManager


# NOTICE: 已弃用，请勿使用
class DeprecatedTaskManagerModel(db.Model):
    """任务管理器备份模型"""
    __tablename__ = "task_manager_model"
    id = db.Column(db.Integer, primary_key=True, autoincrement=True)
    task_manager_data = db.Column(db.LargeBinary(65536))  # TASKMANAGER备份数据
    offline_task_manager_data = db.Column(
        db.LargeBinary(65536)
    )  # OFFLINETASK_MANAGER备份数据
    backup_time = db.Column(db.DateTime)


class TaskManagerModel(db.Model):
    __tablename__ = "wcs_task_manager_cache"
    id = sa.Column(sa.Integer, primary_key=True, autoincrement=True)
    bin = sa.Column(sa.LargeBinary(65536))
    backup_time = sa.Column(sa.DateTime, onupdate=datetime.now)


class WorkspaceManagerModel(db.Model):
    __tablename__ = "wcs_workspace_manager_cache"
    id = sa.Column(sa.Integer, primary_key=True, autoincrement=True)
    bin = sa.Column(sa.LargeBinary(65536))
    backup_time = sa.Column(sa.DateTime, onupdate=datetime.now)


class OrderManagerModel(db.Model):
    __tablename__ = "wcs_order_manager_cache"
    id = sa.Column(sa.Integer, primary_key=True, autoincrement=True)
    bin = sa.Column(sa.LargeBinary)
    backup_time = sa.Column(sa.DateTime, onupdate=datetime.now)


class HistoryTaskModel(db.Model, DBModelMixin["HistoryTaskModel", LiteBaseTask]):
    """
    历史任务表
    """
    __tablename__ = "wcs_history_tasks"
    id = sa.Column(sa.Integer, primary_key=True, autoincrement=True)
    task_id = sa.Column(sa.String(50), comment="任务ID")
    order_id = sa.Column(sa.String(50), default="", comment="订单ID")
    task_type = sa.Column(sa.Enum(TaskType), comment="任务类型")
    task_status = sa.Column(sa.Enum(TaskStatus), comment="任务状态")
    sku_info = sa.Column(sa.JSON, comment="sku的信息")
    target_num = sa.Column(sa.Integer, comment="需要执行的数量")
    done_num = sa.Column(sa.Integer, comment="已完成的数量")
    from_ws = sa.Column(sa.String(50), comment="抓取位ID")
    to_ws = sa.Column(sa.String(50), comment="放置位ID")
    customized_data = sa.Column(sa.JSON, comment="自定义数据")
    start_time = sa.Column(sa.DateTime, comment="开始时间")
    end_time = sa.Column(sa.DateTime, comment="结束时间")
    create_time = sa.Column(
        sa.DateTime,
        index=True,
        default=lambda: datetime.now(),
        comment="创建时间"
    )
    error_code = sa.Column(sa.Integer, default=None, comment="异常码")
    error_msg = sa.Column(sa.String(255), default=None, comment="异常信息")

    __table_args__ = (
        sa.Index("idx_task_id_order_id", task_id, order_id),
    )


class HistoryOrderModel(db.Model, DBModelMixin["HistoryOrderModel", LiteOrder]):
    """
    历史订单表
    """
    __tablename__ = "wcs_history_orders"
    id = sa.Column(sa.Integer, primary_key=True, autoincrement=True)
    order_id = sa.Column(
        sa.String(50),
        nullable=False,
        comment="订单号"
    )
    order_status = sa.Column(sa.Enum(TaskStatus), comment="订单状态")
    customized_data = sa.Column(sa.JSON, comment="自定义数据")
    start_time = sa.Column(sa.DateTime, comment="开始时间")
    end_time = sa.Column(sa.DateTime, comment="结束时间")
    create_time = sa.Column(
        sa.DateTime,
        index=True,
        default=lambda: datetime.now(),
        comment="创建时间"
    )
    error_code = sa.Column(sa.Integer, default=None, comment="异常码")
    error_msg = sa.Column(sa.String(255), default=None, comment="异常信息")

    __table_args__ = (
        sa.Index("idx_order_id", order_id, id, unique=True),
    )



class PalletData(db.Model):
    __tablename__ = "pallet_data"
    id = sa.Column(sa.Integer, primary_key=True, autoincrement=True)
    current_direction = sa.Column(sa.String(50), comment="当前拣配托盘朝向")
    pallet_tote_data = sa.Column(sa.JSON, comment="拣配托盘上物料信息")
    pick_tote_data = sa.Column(sa.JSON, comment="需要抓取的物料信息")
    cache_pallet_tote_data = sa.Column(sa.JSON, comment="缓存托盘上物料信息")
    path =sa.Column(sa.JSON, comment="路径")
    pallet_tote_data_2 = sa.Column(sa.JSON, comment="笼车托盘2上的物料信息")
    pallet_tote_data_3 = sa.Column(sa.JSON, comment="笼车托盘3上的物料信息")
    pallet_tote_data_7 = sa.Column(sa.JSON, comment="混码缓存区7上的物料信息")
    pallet_tote_data_8 = sa.Column(sa.JSON, comment="混码缓存区8上的物料信息")

class CageLayer(db.Model):
    __tablename__ = "cage_layer"
    id = sa.Column(sa.Integer, primary_key=True, autoincrement=True)
    layer_num = sa.Column(sa.Integer, comment="中欧笼车层数")     
    