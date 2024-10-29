import requests
import json,time

def execute(self, inputs, outputs, gvm):
    """ 
    Check if the pick workspace is ready for picking.

    Args:
        Inputs Data:
            None

        Outputs Data:
            None

        Gvm: 
            None
            
    Properties Data:
        comment, (unicode): Default value (u"is_pick_ws_ready").
            The comment of this state. This will show in state's GUI block.
    
        task_id: the id of the current task
        ws_id: the id of pick workspace

    Outcomes:
        0: success
        1: fail
        -1: aborted
        -2: preempted
    """
    self.logger.info("Running {}({})({})".format(self.name, self.unique_id, self.smart_data["comment"]))

    url = "http://127.0.0.1:7002/api/rafcon/is_pick_ws_ready"
    data = {
        "task_id": self.smart_data["task_id"],
        "ws_id": self.smart_data["ws_id"]
        }
    
    max_wait_time = 60
    current_time = time.time()
    info_msg = ''
    while not self.preempted:
        if time.time()-current_time>max_wait_time:          
            from xyz_logistics_hmi_back.utils.utils import send_order_log
            msg = f"等待笼车到位超时"
            send_order_log(message=msg, status=False)
            return "time_out"
        else:
            try:
                response = requests.post(url, json = data).json()
            except requests.exceptions.ConnectionError as e:
                raise Exception("Connection Error: {}".format(e))
            except: 
                raise Exception("Unknown errors occured when requesting response from hmi-back.")
    
            result = response["result"]
            error = response["error"]
            error_message = response["error_message"]
            if result == True:
                self.logger.info(response)
                return "success"
            elif result == False:
                if info_msg!="抓取条件不满足":
                    info_msg="抓取条件不满足"
                    self.logger.info(info_msg)
                    self.logger.info(response)
                continue    
    


