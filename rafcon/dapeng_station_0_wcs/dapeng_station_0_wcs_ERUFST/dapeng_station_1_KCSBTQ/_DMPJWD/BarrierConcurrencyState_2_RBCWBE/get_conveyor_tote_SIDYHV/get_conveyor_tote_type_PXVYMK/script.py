import requests,time
from xyz_io_client.io_client import set_digit_output

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
    
    while not self.preempted:
         
                    
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
            time.sleep(10)        
            return "timeout"            
        try: 
            response = requests.post(url).json()
        except requests.exceptions.ConnectionError as e:
            raise Exception("Connection Error: {}".format(e))
        except: 
            raise Exception("Unknown errors occured when requesting response from hmi-back.")   

        data = response["data"]
        error = int(data["error"])    
        if error == 0:
            tote_type = int(data["tote_type"])
            to_ws = data["to_ws"]         
            outputs["tote_type"] = tote_type
            outputs["place_id"] = to_ws  
            self.logger.info(f"tote is {tote_type},place id is {to_ws}")
            return "success" 
        elif error == 1:
            self.logger.info(f"无法获取输送料箱信息")  
            return "fault" 
        else:                     
            if logger_msg!="未获取到输送线料箱信息":
                self.logger.info(response)
                self.logger.info(f"未获取到输送线料箱信息")
                logger_msg = "未获取到输送线料箱信息"
            continue          

