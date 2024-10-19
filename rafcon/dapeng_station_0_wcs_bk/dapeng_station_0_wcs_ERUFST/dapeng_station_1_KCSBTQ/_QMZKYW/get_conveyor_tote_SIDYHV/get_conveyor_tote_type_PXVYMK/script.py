import requests,time
from xyz_io_client.io_client import set_digit_output

def get_task_status(self):
    url = "http://127.0.0.1:7002/api/rafcon/get_multi_task_status"
    data = {
        }
    try:
        response = requests.post(url, json = data).json()
    except requests.exceptions.ConnectionError as e:
        raise Exception("Connection Error: {}".format(e))
    except: 
        raise Exception("Unknown errors occured when requesting response from hmi-back.") 
     
    task_status = response["task_status"]  
    return task_status

def execute(self, inputs, outputs, gvm):
    
    url = "http://127.0.0.1:7002/api/rafcon/get_conveyor_tote"
    #red
    set_digit_output("2",65031,0)
    #yellow
    set_digit_output("2",65032,0)
    #green
    set_digit_output("2",65033,1)
    #buzzer
    set_digit_output("2",65034,0)      
    current_time = time.time()
    max_time = 60
    logger_msg = ""
    pallet_tote_data_7 = inputs["pallet_tote_data_7"]
    pallet_tote_data_8 = inputs["pallet_tote_data_8"]
    
    while not self.preempted:
        
        task_status = get_task_status(self)
        if task_status==4 or task_status==2:
            self.logger.info("Task type is terminate")
            if pallet_tote_data_7:
                self.logger.info("Task type is terminate and pallet_tote_data_7 is True")
                outputs["cache_id"] = "7"
                outputs["place_id"] = pallet_tote_data_7["to_ws"]
                return "task_cache"
            elif pallet_tote_data_8:
                self.logger.info("Task type is terminate and pallet_tote_data_8 is True")
                outputs["cache_id"] = "8"
                outputs["place_id"] = pallet_tote_data_8["to_ws"]   
                return "task_cache" 
            else:             
                return "task_finish"  
                    
        if time.time()-current_time>max_time:
            from xyz_logistics_hmi_back.utils.utils import send_order_log
            self.logger.info(f"空箱回收获取输送线料箱信息超时")
            msg = f"空箱回收获取输送线料箱信息超时"
            send_order_log(message=msg, status=False)
            #red
            set_digit_output("2",65031,1)
            #yellow
            set_digit_output("2",65032,0)
            #green
            set_digit_output("2",65033,0)
            #buzzer
            set_digit_output("2",65034,1)           
            return "timeout"            
        try: 
            response = requests.post(url).json()
        except requests.exceptions.ConnectionError as e:
            raise Exception("Connection Error: {}".format(e))
        except: 
            raise Exception("Unknown errors occured when requesting response from hmi-back.")   

        data = response["data"]
        error = int(data["error"])    
        if error!=0:
            if logger_msg!="未获取到输送线料箱信息":
                self.logger.info(response)
                self.logger.info(f"未获取到输送线料箱信息")
                logger_msg = "未获取到输送线料箱信息"
            continue
        else:
            tote_type = int(data["tote_type"])
            to_ws = data["to_ws"]
            outputs["tote_type"] = tote_type
            outputs["place_id"] = to_ws  
            self.logger.info(f"tote is {tote_type},place id is {to_ws}")
            return "success"            

