import json
import requests
def undate_task_status(self,task_status):
    url = "http://127.0.0.1:7002/api/rafcon/update_multi_task_status"
    data = {
        "task_status":task_status
        }    
    try:
        response = requests.post(url, json = data).json()
    except requests.exceptions.ConnectionError as e:
        raise Exception("Connection Error: {}".format(e))
    except: 
        raise Exception("Unknown errors occured when requesting response from hmi-back.")  
    self.logger.info("response: {}".format(response))
    return True   
    
def execute(self, inputs, outputs, gvm):
    self.logger.info("Hello {}".format(self.name))
    undate_task_status(self,self.smart_data["task_status"])
    return "success"
