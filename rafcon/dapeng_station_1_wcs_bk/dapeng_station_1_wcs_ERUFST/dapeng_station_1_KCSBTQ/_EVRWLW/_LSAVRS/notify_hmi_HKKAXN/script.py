from rafcon.xyz_exception_base import XYZExceptionBase
from xyz_logistics_hmi_back.utils.utils import send_order_log

def execute(self, inputs, outputs, gvm):
    """ 
    To throw a error of other state's exception.

    Args:
        Inputs Data:
            None

        Outputs Data:
            None

        Gvm: 
            None
            

    Properties Data:
        https://xyz-robotics.atlassian.net/wiki/spaces/depalletizeproduct/pages/508329985#%E9%80%9A%E7%9F%A5HMI%EF%BC%88notify_hmi%EF%BC%89
    Outcomes:
        0: success
        -1: aborted
        -2: preempted
    """
    self.logger.info("Running {}({})({})".format(self.name, self.unique_id, self.smart_data["comment"]))
    
    is_notify_error_mode = self.smart_data["is_notify_error_mode"]
    if is_notify_error_mode:
        gvm.set_variable("ERROR", True)
        error_code = self.smart_data['notify_error_mode']['error_code']
        raise XYZExceptionBase(error_code, error_msg="")
    else:
        msg = self.smart_data["notify_msg_mode"]["msg"]
        status = self.smart_data["notify_msg_mode"]["status"]
        send_order_log(message=msg, status=status)
    return "success"