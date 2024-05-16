from pydantic import Field

from wcs_adaptor.entity import LiteBaseTask
from typing import Optional
from typing import List
class DPTTask(LiteBaseTask):
    """拆码垛任务类.

    在 `wcs_adaptor.entity.LiteBaseTask` 的基础上添加了拆码垛任务的特有属性.
        * from_ws: 抓取位, 必填
        * to_ws: 放置位, 必填

    使用 `default` 参数设置默认值
        from_ws: str = Field(default="0", description="抓取位")

    Attributes:
        task_id(str): 任务ID.
        order_id(str): 订单ID, 默认为空.
        sku_info(Optional[SkuInfo]): SKU信息.
        task_status(TaskStatus): 任务状态.
        task_type(TaskType): 任务类型.
        task_type(TaskType): 任务类型.
        task_status(TaskStatus): 任务状态.
        start_time(datetime): 任务开始时间.
        end_time(datetime): 任务结束时间.
        error_code(int): 错误码.
        error_msg(str): 错误信息.
        done_num(int): 已完成数量.
        target_num(int): 理论数量.
        sku_info(SkuInfo): sku信息.
        customized_data(dict): 自定义数据.
        from_ws(str): 抓取空间.
        to_ws(str): 放置空间.

    Methods:
        report_pick_num: 报告抓取数量
        is_finished: 判断任务是否已完成
        start: 开始任务
        finish: 完成任务
        terminate: 终止任务
        reset: 重置任务状态
    """
    from_ws: str = Field(description="抓取空间ID")
    to_ws: str = Field(description="放置空间ID")
    order_done_num: int = Field(default=0, description="所在订单的完成数量")
    pallet_tote_data: Optional[dict] = Field(default={},description="托盘料箱数据")
    pick_tote_data: Optional[dict] = Field(default={},description="抓取料箱数据")
    pallet_clear_list: list = Field(default=[], description="已更换的托盘")
    from_pallet_tote_data: Optional[dict] = Field(default={},description="合托任务拼托副（被拆）数据")
    to_pallet_tote_data: Optional[dict] = Field(default={},description="合托任务拼托主（码）工作空间数据")
    
        
        