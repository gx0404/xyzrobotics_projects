import requests
import json
from rafcon.xyz_exception_base import XYZExceptionBase


def report_task_ending(self):
    if self.smart_data["report_task_ending_enable"]:
        url_ending = "http://127.0.0.1:7002/api/rafcon/report_task_ending"
        data_ending = {
            "task_id": self.smart_data["task_id"]
            }
        try: 
            response_ending = requests.post(url_ending, json = data_ending).json()
            self.logger.info(response_ending)
            if response_ending['code'] != 0:
                self.logger.error("report_task_ending服务错误,后端xyz-logistics-hmi-back版本需要在V1.5.0以上。版本在V1.5.0以下的，则report_task_ending_enable设置为False")
        except: 
            self.logger.error("report_task_ending服务错误")

def execute(self, inputs, outputs, gvm):
    """ 
    Report the status of the current task.

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
        pick_num: the number of items picked in this cycle
        is_depal_pallet_empty: whether the pallet for depalletizing is empty
        is_pal_pallet_full: whether the pallet for palletizing is empty
        error: 0 for success and 1 for abnormal status
        error_message: explanation of the error

    Outcomes:
        0: success
        -1: aborted
        -2: preempted
    """
    self.logger.info("Running {}({})({})".format(self.name, self.unique_id, self.smart_data["comment"]))

    if self.smart_data["is_depal_pallet_empty"] & self.smart_data["is_pal_pallet_full"]:
        raise XYZExceptionBase(error_code="99998", error_msg="parameter setting error")

    url = "http://127.0.0.1:7002/api/rafcon/report_task_status"
    data = {
        "task_id": self.smart_data["task_id"],
        "pick_num": self.smart_data["pick_num"],
        "drop_num": self.smart_data["drop_num"],
        "is_depal_pallet_empty": self.smart_data["is_depal_pallet_empty"],
        "is_pal_pallet_full": self.smart_data["is_pal_pallet_full"],
        "place_id":inputs["place_id"],
        "error": 0,
        "error_message": "success"
        }
    try: 
        response = requests.post(url, json = data).json()
    except requests.exceptions.ConnectionError as e:
        raise Exception("Connection Error: {}".format(e))
    except: 
        raise Exception("Unknown errors occured when requesting response from hmi-back.")
    self.logger.info(response)

    if self.smart_data["drop_num"] != 0:
        return "dropbox"

    # 兼容后端没有提供is_task_finished的情况，即XLH V1.4.0之前（包含V1.4.0）的版本
    try: 
        is_order_finished = response["data"]["is_order_finished"]
        if is_order_finished:
            report_task_ending(self)
            return "order_finished"
        is_task_finished = response["data"]["is_task_finished"]
        if is_task_finished:
            report_task_ending(self)
            self.logger.info(f"任务被wcs终止,但需要继续执行")
            return "success"
    except: 
        raise "report error"

    return "success"
