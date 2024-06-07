import time

def execute(self, inputs, outputs, gvm):
    """ 
    Wait for event to be set.

    Args:
    
        Inputs Data:
            None

        Outputs Data:
            None

        Gvm: 
            None

    Properties Data:
    
        comment (str): Default value("check_home").
            注释
        event_name(str): Default value("event1").
            信号的名字
        timeout(float): Default value(5.0).
            等待时间， 单位秒

    Outcomes:
        1: timeout
        0: success
        -1: aborted
        -2: preempted
    """
    self.logger.info("Running {}({})({})".format(self.name, self.unique_id, self.smart_data["comment"]))
    
    wait_start_time = time.time()
    time_limit_seconds = self.smart_data["timeout"]
    while not self.preempted:
        if gvm.variable_exist("ERROR"):
            return "error"
        if (time.time() - wait_start_time) > time_limit_seconds:
            return "timeout"
        
        if gvm.variable_exist("EVENT-" + self.smart_data["event_name"]):
            gvm.delete_variable("EVENT-" + self.smart_data["event_name"])
            break
        
        self.preemptive_wait(0.1)
        
    return "success"