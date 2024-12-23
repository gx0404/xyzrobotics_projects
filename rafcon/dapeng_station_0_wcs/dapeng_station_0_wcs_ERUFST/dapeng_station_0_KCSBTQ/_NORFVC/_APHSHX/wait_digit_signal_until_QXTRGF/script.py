import time
from xyz_io_client.io_client import get_digit_input

def cont_check(device_id, port_ids, cont_check_time, digit_signals, expected_values):
    start_time = time.time()
    while (time.time() - start_time) <= cont_check_time:
        new_digit_signals = []
        for port, expected_value in zip(port_ids, expected_values):
            new_digit_signals.append(get_digit_input(device_id, port, expected_value))
        if new_digit_signals == digit_signals:
            continue
        else:
            return False, new_digit_signals
    
    return True, digit_signals

def execute(self, inputs, outputs, gvm):
    """
    get_digit_signal_continuously
    THis state is aim to make sure that the target siganl we get is a stable siganl not a fake signal

    Args:
        Inputs Data:
            None

        Outputs Data:
            None

        Gvm: 
            None
            

    Properties Data:
        comment, (unicode): Default value (u"wait_analog_signal_until").
            The comment of this state. This will show in state's GUI block.
        
        device_id, (unicode):  Default value (u"2").
        
        ports, (list of int): Default value ([0]).
            The start port for reading.

        sleep_interval, (float): Default value (0.1). unit: second. 
            Specify the sleep interval during waiting.

        target_values, (list with elements 0 or 1): Default value ([1]). 
            The reference value to compare with IO signal's value.

        expected_values, (list with elements 0 or 1): Default value ([1]).
            When the io_server works on the simulation mode(-v sim), you can use virtual IO device, but you need to specify 
            the expected_data as the function returns.

        timeout: (float): Default value (1e+300). unit: second. Wait the signal to target_value. If timeout, will return 
            outcome: timeout
        cont_check_time: (float): the max time that target signal keep
    """
    self.logger.info("Running {}({})({})".format(self.name, self.unique_id, self.smart_data["comment"]))
    start_time = time.time()
    if self.smart_data["timeout"] < self.smart_data["cont_check_time"]:
        raise Exception("最大等待时间应大于最大监测持续时间")
    
    while not self.preempted:
        #if gvm.variable_exist("ERROR"):
            #return "timeout"
        if (time.time() - start_time) > self.smart_data["timeout"]:
            from xyz_logistics_hmi_back.utils.utils import send_order_log
            msg = f"等待输送线超时"
            send_order_log(message=msg, status=False)
            from xyz_io_client.io_client import set_digit_output
            #red
            set_digit_output("2",65031,1)
            #yellow
            set_digit_output("2",65032,0)
            #green
            set_digit_output("2",65033,0)
            #buzzer
            set_digit_output("2",65034,1)          
            from rafcon.xyz_exception_base import XYZExceptionBase
            raise XYZExceptionBase("E0800", "夹具异常：检测或控制失败")
        digit_signals = []
        for port, expected_value in zip(self.smart_data["ports"], self.smart_data["expected_values"]):
            digit_signals.append(get_digit_input(self.smart_data["device_id"], port, expected_value))
        
        if digit_signals == self.smart_data["target_values"]:
            check_status, signal_status = cont_check(self.smart_data["device_id"], self.smart_data["ports"], self.smart_data["cont_check_time"], digit_signals, self.smart_data["expected_values"])
            if check_status:
                return "success"
            
        self.preemptive_wait(self.smart_data["sleep_interval"])
        
        
