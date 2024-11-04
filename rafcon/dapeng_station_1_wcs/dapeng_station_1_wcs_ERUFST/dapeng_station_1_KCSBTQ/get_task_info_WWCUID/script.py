from logging import error
import requests
import json
import time
from rafcon.xyz_exception_base import XYZExceptionBase
from xyz_io_client.io_client import set_digit_output

def execute(self, inputs, outputs, gvm):
    """ 
    Get task info from xyz_hmi_back.

    Args:
        Inputs Data:
            None

        Outputs Data:
            task_info (dict): Default value (None).
                Get the task info(result, task_id, sku_info, max_num, undone_num, from, to, task_type, clear_from_ws, clear_to_ws, etc) from xyz_hmi_back.

        Gvm: 
            None
            
    Properties Data:
        详见此模块的说明文档


    Outcomes:
        0: success
        1: fail
        2: timeout
        -1: aborted
        -2: preempted
    """
    self.logger.info("Running {}({})({})".format(self.name, self.unique_id, self.smart_data["comment"]))
    
    #设置三色灯变成黄色
    if self.smart_data["tri-color_light_setting"]["enable"]:
        try:
            #red
            #set_digit_output("2",65031,0)
            #buzzer
            #set_digit_output("2",65034,0)
            #yellow
            set_digit_output("2",65032,1)
            #green
            set_digit_output("2",65033,0)
            # set_digit_output(self.smart_data["tri-color_light_setting"]["device_id"], 
            #     self.smart_data["tri-color_light_setting"]["port"], 
            #     self.smart_data["tri-color_light_setting"]["value_yellow"])
        except:
            raise XYZExceptionBase("70018", "The communication between the industrial computer and the PLC is disconnected or the address bit of the operation is wrong.")

    url = "http://127.0.0.1:7002/api/rafcon/get_task_info"
    is_timeout_mode = self.smart_data["is_timeout_mode"]
    data = {}
    if is_timeout_mode:
        start_time = time.time()
        while not self.preempted:
            if gvm.variable_exist("ERROR"):
                return "timeout"
            if (time.time() - start_time) >  self.smart_data["setting_timeout_mode"]["timeout"]:
                return "timeout"

            try:
                #response = requests.post(url).json()
                response = requests.post(url, json = data).json()
            except requests.exceptions.ConnectionError as e:
                raise Exception("Connection Error: {}".format(e))
            except: 
                raise Exception("Unknown errors occured when requesting response from hmi-back.")

            result = response["result"]
            if result == True:
                break
            
            self.preemptive_wait( self.smart_data["setting_timeout_mode"]["sleep_interval"])
    else:        
        try:
            response = requests.post(url).json()
        except requests.exceptions.ConnectionError as e:
            raise Exception("Connection Error: {}".format(e))
        except: 
            raise Exception("Unknown errors occured when requesting response from hmi-back.")

        error = response["error"]
        if error == 1:
            raise Exception("rafcon get_task_info failed: {}".format(response["error_message"]))

        result = response["result"]
        if result == False:
            return "fail"
        
    task_info = response
    outputs["task_info"] = task_info
    outputs["max_num"] = task_info["max_num"]
    outputs["undone_num"] = task_info['undone_num']
    outputs["task_id"] = str(task_info["task_id"])
    outputs["sku_info"] = task_info["sku_info"]
    outputs["pick_workspace_id"] = str(task_info["from"])
    outputs["place_workspace_id"] = str(task_info["to"])
    self.logger.info(response)
    # map_box
    client = requests.session()
    #通过过滤条件查询map box
    data =  {
            "page": 1,
            "page_size": 20,
            "single": False,
            "filters": [
                {
                "op": "eq",
                "field": "normal_length",
                "value": task_info["sku_info"]["length"]*1000
                },
                {
                "op": "eq",
                "field": "normal_width",
                "value": task_info["sku_info"]["width"]*1000
                },
                                {
                "op": "eq",
                "field": "normal_height",
                "value": task_info["sku_info"]["height"]*1000
                },

            ],
            "sort": [
                {
                "field": "real_length",
                "collation": 0
                },
                {
                "field": "serial_number",
                "collation": -1
                }
            ]
            }			

    map_box_url = "http://127.0.0.1:7002/api/dpt/map_box"
    try:
        res = client.get(map_box_url, json = data)
        res_data = res.json()
        self.logger.info(res_data)
    except requests.exceptions.ConnectionError as e:
        raise Exception("Connection Error: {}".format(e))
    except: 
        raise Exception("Unknown errors occured when requesting response from hmi-back.")

    code = res_data["code"]
    if code != 0:
        raise Exception("search map_box failed: {}".format(res_data["msg"]))

    if len(res_data["data"]) > 1:
        raise Exception("映射的纸箱数目超过1个，映射表错误或者过滤查询条件错误")

    extra_dict = {}
    if res_data["data"]:
        map_box_data = res_data["data"][0]
        if map_box_data["extra1"]:
            extra_dict["extra1"] = map_box_data["extra1"]
        if map_box_data["extra2"]:
            extra_dict["extra2"] = map_box_data["extra2"]
        if map_box_data["extra3"]:
            extra_dict["extra3"] = map_box_data["extra3"]
        if map_box_data["extra4"]:
            extra_dict["extra4"] = map_box_data["extra4"]

    outputs["extra"] = extra_dict

    #设置三色灯变成绿色
    if self.smart_data["tri-color_light_setting"]["enable"]:
        try:
            #red
            set_digit_output("2",65031,0)
            #buzzer
            set_digit_output("2",65034,0)
            #yellow
            set_digit_output("2",65032,0)
            #green
            set_digit_output("2",65033,1)            
            # set_digit_output(self.smart_data["tri-color_light_setting"]["device_id"], 
            #     self.smart_data["tri-color_light_setting"]["port"], 
            #     self.smart_data["tri-color_light_setting"]["value_green"])
        except:
            raise XYZExceptionBase("70018", "The communication between the industrial computer and the PLC is disconnected or the address bit of the operation is wrong.")
    outputs["pallet_clear_list"] = task_info["pallet_clear_list"] 
    layer_num = task_info["layer_num"]
    gvm.set_variable("layer_num", layer_num, per_reference=False)    
    if task_info["lower_speed"]:
        gvm.set_variable("lower_speed", True, per_reference=True) 
    else:
        gvm.set_variable("lower_speed", False, per_reference=True)
    if task_info["task_type"]==0:
        outputs["pallet_clear_list"] = task_info["pallet_clear_list"]+["0","4"] 
        self.logger.info(f"执行拣配任务")
        outputs["lower_layer"] = task_info["lower_layer"]
        return "depal"
    elif task_info["task_type"]==1:
        outputs["pallet_clear_list"] = task_info["pallet_clear_list"]+["2","3"] 
        outputs["customized_data"] = task_info["customized_data"]
        self.logger.info(f"执行笼车单码任务")
        return "pal"      
    elif task_info["task_type"]==3:
        self.logger.info(f"执行输送线混码任务")
        return "multi_pal"      
    elif task_info["task_type"]==10:
        outputs["pallet_clear_list"] = task_info["pallet_clear_list"]+["0","1"] 
        self.logger.info(f"执行合托任务")
        return "merge"                
    else:
        raise "无效的任务类型"