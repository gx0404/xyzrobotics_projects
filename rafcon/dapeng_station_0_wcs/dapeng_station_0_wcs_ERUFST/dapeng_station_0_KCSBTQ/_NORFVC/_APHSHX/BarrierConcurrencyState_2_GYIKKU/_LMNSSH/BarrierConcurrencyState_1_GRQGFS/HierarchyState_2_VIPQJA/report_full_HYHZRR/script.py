import requests
import json
from rafcon.xyz_exception_base import XYZExceptionBase
import time

def execute(self, inputs, outputs, gvm):
    """ 
    Report current task finish to wcs.

    Args:
        Inputs Data:
            None

        Outputs Data:
            None

        Gvm: 
            None
            
    Properties Data:
        comment, (unicode): Default value (u"from_extern_service").
            The comment of this state. This will show in state's GUI block.

        task_id: the id of the current task

    Outcomes:
        0: success
        -1: aborted
        -2: preempted
    """
    self.logger.info("Running {}({})({})".format(self.name, self.unique_id, self.smart_data["comment"]))
    trajectory = inputs["trajectory"]
    place_id = trajectory["grasp_plan"].to_workspace_id
    is_pal_pallet_full = inputs["is_pal_pallet_full"]
    if is_pal_pallet_full:
        time.sleep(1)
        self.logger.info("已放满,回报放满")
        url = "http://127.0.0.1:7002/api/rafcon/report_depal_full"
        data = {
            "place_id":place_id
            }
        try: 
            response = requests.post(url, json = data).json()
        except requests.exceptions.ConnectionError as e:
            raise Exception("Connection Error: {}".format(e))
        except: 
            raise Exception("Unknown errors occured when requesting response from hmi-back.")
        self.logger.info(response)  
    return "success"
