#  -*- coding: utf-8 -*-
#  Copyright (c) XYZ Robotics Inc. - All Rights Reserved
#  Unauthorized copying of this file, via any medium is strictly prohibited
#  Proprietary and confidential
#  Author: Yuhang Wu <yuhang.wu@xyzrobotics.ai>, 2022/8/12 上午8:49
from flask import Blueprint, current_app

import wcs_adaptor
from apps import XYZBaseError, openapi
from apps.base_app.flask_hook import req_log as catch_log
from apps.responses import StdResponse
from wcs_adaptor.models import PalletData
from wcs_adaptor.models import HistoryTaskModel
from datetime import datetime
bp = Blueprint("ext", __name__, url_prefix="/api/ext")


@bp.route("/get_current_task")
@catch_log()
@openapi.api_doc(tags=["XLHB", "扩展接口"], summary="获取当前任务")
def get_current_task():
    """获取正在执行的任务.""" 
     
    try:
        #import ipdb;ipdb.set_trace() 
        all_history_tasks = HistoryTaskModel.query.all()
        all_history_tasks = all_history_tasks[-800:]
        now_day = datetime.now().strftime("%Y-%m-%d")
        today_tasks = []
        for history_task in all_history_tasks:
            if history_task.end_time.strftime("%Y-%m-%d")==now_day:   
                today_tasks.append(history_task)
                    
                
        #import ipdb;ipdb.set_trace()
        running_time = 0
        for today_task in today_tasks:
            if today_task.create_time.strftime("%Y-%m-%d")!=now_day:
                task_hour = today_task.end_time.hour+(24-today_task.create_time.hour)
            else:    
                task_hour = today_task.end_time.hour-today_task.create_time.hour  
            task_minute = today_task.end_time.minute-today_task.create_time.minute
            task_second = today_task.end_time.second-today_task.create_time.second
            running_time+= task_hour*3600+task_minute*60+task_second
        running_time = running_time/3600
        now_time = (datetime.now().hour*3600+datetime.now().minute*60+datetime.now().second)/3600  
        
        check_tasks_0 = []
        check_tasks_1 = []
        
        #点击的时间
        #白班
        if datetime.now().hour < 20 and datetime.now().hour >= 8:
            for history_task in all_history_tasks:
                if history_task.end_time.strftime("%Y-%m-%d")==now_day: 
                    if history_task.end_time.hour >= 8 and history_task.end_time.hour <20:
                        if history_task.task_id not in [i.task_id for i in check_tasks_0]: 
                            check_tasks_0.append(history_task) 
                    if history_task.end_time.hour>=0 and history_task.end_time.hour <8:
                        if history_task.task_id not in [i.task_id for i in check_tasks_1]:                        
                            check_tasks_1.append(history_task)
                                  
                if history_task.end_time.day==datetime.now().day-1 and\
                    history_task.end_time.hour <= 23 and history_task.end_time.hour >=20:  
                    if history_task.task_id not in [i.task_id for i in check_tasks_1]:                        
                        check_tasks_1.append(history_task)                    
                               
        #夜班             
        elif datetime.now().hour >= 20 and datetime.now().hour <= 23:
            for history_task in all_history_tasks:
                if history_task.end_time.strftime("%Y-%m-%d")==now_day and history_task.end_time.hour >= 8\
                    and history_task.end_time.hour <20:
                    if history_task.task_id not in [i.task_id for i in check_tasks_0]: 
                        check_tasks_0.append(history_task)                         
                if history_task.end_time.strftime("%Y-%m-%d")==now_day and history_task.end_time.hour >= 20\
                and history_task.end_time.hour <= 23:
                    if history_task.task_id not in [i.task_id for i in check_tasks_1]:                        
                        check_tasks_1.append(history_task)                       

                                  
        
        elif datetime.now().hour >= 0 and datetime.now().hour < 8:
            for history_task in all_history_tasks:
                if history_task.end_time.day==datetime.now().day-1 and history_task.end_time.hour >=8\
                    and history_task.end_time.hour <20:
                    if history_task.task_id not in [i.task_id for i in check_tasks_0]: 
                        check_tasks_0.append(history_task) 
                if history_task.end_time.day==datetime.now().day-1 and history_task.end_time.hour >=20\
                    and history_task.end_time.hour<=23:
                    if history_task.task_id not in [i.task_id for i in check_tasks_1]:                        
                        check_tasks_1.append(history_task)                                                    
                if history_task.end_time.strftime("%Y-%m-%d")==now_day and history_task.end_time.hour >= 0\
                    and history_task.end_time.hour <8:
                    if history_task.task_id not in [i.task_id for i in check_tasks_1]:                        
                        check_tasks_1.append(history_task)   
        
        #import ipdb;ipdb.set_trace()               
                                    
        
        num_1 = 0
        num_2 = 0
        for _task in check_tasks_0:
            num_1+= len(_task.customized_data)
        for _task in check_tasks_1:
            num_2+= len(_task.customized_data)  
                
                    
        model = PalletData.query.first() 
        pick_tote_data = model.pick_tote_data                    
        if task := wcs_adaptor.manager.task_manager.first():
            try:
                sku_info = task.sku_info
                sku_dimension = [sku_info.length,sku_info.width,sku_info.height]
                sku_dimension = list(map(lambda x:round(x,3),sku_dimension))
                #import ipdb;ipdb.set_trace()
                if sku_dimension==[400,300,230]:
                    sku_type = "中欧"
                elif sku_dimension==[600,400,230]: 
                    sku_type = "大欧"
                if task.task_type == 0:
                    task_type = "拣配任务"
                elif task.task_type == 10:
                    task_type = "合托任务"       
                elif task.task_type == 3:
                    task_type = "输送线空箱回收"   
                elif task.task_type == 1:
                    task_type = "笼车空箱回收"    
                data = {
                    "当天机械臂执行任务总时间(含任务异常)":f"{round(running_time,2)}小时",
                    "执行任务时间/当天24小时":f"{round(running_time/now_time*100,2)}%",
                    "白班拣单数量(8:00-20:00)":num_1,
                    "夜班拣单数量(20:00-8:00)":num_2,
                    "任务ID": task.task_id,
                    "已完成数量": task.done_num,
                    "目标数量": task.target_num,
                    "任务类型":task_type,
                    "料箱类型":sku_type,
                    "目标箱":pick_tote_data,
                    "sku_info": {
                        "length": task.sku_info and task.sku_info.length,
                        "width": task.sku_info and task.sku_info.width,
                        "height": task.sku_info and task.sku_info.height,
                        "weight": task.sku_info and task.sku_info.weight,
                    },
                }
            except AttributeError as err:
                raise XYZBaseError(error_message="获取任务对象的属性失败") from err

        else:
            data = {
                    "当天机械臂执行任务总时间(含任务异常)":f"{round(running_time,2)}小时",
                    "执行任务时间/当天24小时":f"{round(running_time/now_time*100,2)}%",
                    "白班拣单数量(8:00-20:00)":num_1,
                    "夜班拣单数量(20:00-8:00)":num_2,
            }
    except:
        if task := wcs_adaptor.manager.task_manager.first():
            sku_info = task.sku_info     
            data = {
                "任务ID": task.task_id,
                "已完成数量": task.done_num,
                "目标数量": task.target_num,
                "sku_info": {
                    "length": task.sku_info and task.sku_info.length,
                    "width": task.sku_info and task.sku_info.width,
                    "height": task.sku_info and task.sku_info.height,
                    "weight": task.sku_info and task.sku_info.weight,
                },
            }  
        else:
            data = {
                
            }
    return StdResponse(data=data)


@bp.route("/remove_current_task", methods=["POST"])
@catch_log()
@openapi.api_doc(tags=["XLHB", "扩展接口"], summary="移除当前任务")
def remove_current_task():
    """移除正在执行的任务."""
    if current_app.status == "ready":
        raise XYZBaseError(error_message="任务进行中，无法移除任务")
    tm = wcs_adaptor.manager.task_manager
    om = wcs_adaptor.manager.order_manager
    if order := om.first():
        # 从订单管理器中移除订单, 并移除与该订单相关的所有任务
        om.remove(order)
        for task in order.tasks:
            tm.remove(task)
    else:
        # 只有任务管理器中有任务时才移除
        if task := tm.first():
            tm.remove(task)
    return StdResponse()
