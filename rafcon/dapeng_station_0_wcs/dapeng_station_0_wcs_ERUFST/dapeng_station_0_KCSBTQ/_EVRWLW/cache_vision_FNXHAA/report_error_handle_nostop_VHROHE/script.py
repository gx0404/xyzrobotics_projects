import requests
import json
from rafcon.xyz_exception_base import XYZExceptionBase
from xyz_io_client.io_client import set_digit_output

def execute(self, inputs, outputs, gvm):
    """ 
    Send error to xyz hmi back. You need install xyz-hmi-back before use it.

    This state use to send the exception instance to xyz-hmi-back. 

    Basically, we use this state as the error_handler of other state machines through wrap all other state machines as a 
    hierarchy state, and we connect the hierarchy state's outcome -1(aborted, red outcome port) to error_handler's income.

    You can image that this state "try: except" all other state machines's exception and catch the exception instance automatically
    to the input data: error. 

    Args:
        Inputs Data:
            error (object): Default value (None).
                The error exception instance automatically catch by rafcon.

        Outputs Data:
            None

        Gvm: 
            None
            

    Properties Data:
        comment, (unicode): Default value (u"error_handler").
            The comment of this state. This will show in state's GUI block.
        url, (str): Default value ("http://127.0.0.1:7002/api/rafcon/error_handle_nostop").

    Outcomes:
        0: success
        -1: aborted
        -2: preempted
    """
    self.logger.info("Running {}({})({})".format(self.name, self.unique_id, self.smart_data["comment"]))


    #向前后端汇报错误
    url = self.smart_data["url"]
    error_inst = inputs["error_info"]
    self.logger.info(error_inst)

    error_info = {
            "error": '99999',
            "data":{
            "error_msg": 'program inner error',
            "zh_msg": "",
            "error_code": '99999',
            "tip": ""
            }
        }

    error_info = inputs["error_info"]     
    try:
        response = requests.post(url, json = error_info)
    except requests.exceptions.ConnectionError as e:
        raise Exception("Connection Error: {}".format(e))
    except: 
        raise Exception("Unknown errors occured when requesting response from xyz-hmi-back.")
    self.logger.info(response.json())
    return "success"

