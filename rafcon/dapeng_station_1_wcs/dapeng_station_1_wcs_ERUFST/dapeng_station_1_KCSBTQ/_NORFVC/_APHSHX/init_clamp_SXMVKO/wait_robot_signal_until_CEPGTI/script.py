import time
from xyz_motion import RobotDriver

def cont_check(port_ids, cont_check_time, robot_signal, rob_driver):
    start_time = time.time()
    while (time.time() - start_time) <= cont_check_time:
        new_robot_signal = list(map(rob_driver.get_digital_input, port_ids))
        if new_robot_signal == robot_signal:
            continue
        else:
            return False, new_robot_signal
    return True, robot_signal

def execute(self, inputs, outputs, gvm):
    """ 
    Wait a robot digit signal until target value. 


    Args:
        Inputs Data:
            None

        Outputs Data:
            None

        Gvm: 
            None
            

    Properties Data:
        comment, (unicode): Default value (u"wait_robot_signal_until").
            The comment of this state. This will show in state's GUI block.
        
        robot_id, (unicode):  Default value (u"0").
            Which robot you want to control.
        
        port_ids, (list of int): Default value ([0]).
            Which port you want to control.

        sleep_interval, (float): Default value (0.1). unit: second. 
            Specify the sleep interval during waiting.

        target_values, (list with elements 0 or 1): Default value ([1]). 
            The reference values to compare with robot signal's value.

        expected_values, (list with elements 0 or 1): Default value ([1]).
            If robot is in simulation, the expected port value from robot.

        timeout: (float): Default value (1e+300). unit: second. Wait the signal to target_value. If timeout, will return 
            outcome: timeout
        cont_check_time, (float): Defaultvalue (0).
            the time that check io continuously


    Raises:
        Exception: invalid input of target_value, expected_value

    Outcomes:
        0: success
        1: timeout
        -1: aborted
        -2: preempted
    """
    self.logger.info("Running {}({})({})".format(self.name, self.unique_id, self.smart_data["comment"]))
    start_time = time.time()
    robot_id = int(str(self.smart_data["robot_id"]))
    rob_driver = RobotDriver(robot_id)

    if self.smart_data["timeout"] < self.smart_data["cont_check_time"]:
        raise Exception("最大等待时间应大于最大监测持续时间")

    while not self.preempted:
        #if gvm.variable_exist("ERROR"):
            #return "timeout"
        if (time.time() - start_time) > self.smart_data["timeout"]:
            from xyz_logistics_hmi_back.utils.utils import send_order_log
            msg = f"等待挡边缩回信号超时,信号地址为{self.smart_data['port_ids']}"
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
            raise XYZExceptionBase("E0800", error_msg="夹具异常：检测或控制失败")
        if rob_driver.get_robotstatus()["simulation"]:
            expected_values = self.smart_data["target_values"]
            if expected_values == self.smart_data["target_values"]:
                return "success"
        else:
            expected_values = list(map(rob_driver.get_digital_input, self.smart_data["port_ids"]))
            
        if expected_values == self.smart_data["target_values"]:
            check_status, signal_status = cont_check(self.smart_data["port_ids"], self.smart_data["cont_check_time"], expected_values, rob_driver)
            if check_status:
                return "success"
        self.preemptive_wait(self.smart_data["sleep_interval"])