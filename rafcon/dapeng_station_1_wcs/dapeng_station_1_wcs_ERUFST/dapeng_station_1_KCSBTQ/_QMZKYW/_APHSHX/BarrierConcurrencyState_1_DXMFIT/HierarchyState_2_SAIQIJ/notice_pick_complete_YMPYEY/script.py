import requests,time
from xyz_io_client.io_client import set_digit_output

def execute(self, inputs, outputs, gvm):
    
    url = "http://127.0.0.1:7002/api/rafcon/pick_complete"
    
    current_time = time.time()
    max_time = 30
    logger_msg = ""
    #red
    set_digit_output("2",65031,0)
    #yellow
    set_digit_output("2",65032,0)
    #green
    set_digit_output("2",65033,1)
    #buzzer
    set_digit_output("2",65034,0)  
    
    while not self.preempted:
        if time.time()-current_time>max_time:
            from xyz_logistics_hmi_back.utils.utils import send_order_log
            self.logger.info(f"通知抓取完成信息超时")
            msg = f"通知抓取完成信息超时"
            send_order_log(message=msg, status=False)
            # #red
            # set_digit_output("2",65031,1)
            # #yellow
            # set_digit_output("2",65032,0)
            # #green
            # set_digit_output("2",65033,0)
            # #buzzer
            # set_digit_output("2",65034,1)           
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
            if logger_msg!="通知抓取完成异常":
                self.logger.info(response)
                self.logger.info(f"通知抓取完成异常")
                logger_msg = "通知抓取完成异常"
            continue
        else:
            self.logger.info(response)
            self.logger.info(f"已通知wcs抓取完成")
            return "success"

