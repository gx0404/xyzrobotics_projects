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

    Outcomes:
        0: success
        -1: aborted
        -2: preempted
    """
    self.logger.info("Running {}({})({})".format(self.name, self.unique_id, self.smart_data["comment"]))
    
    gvm.set_variable("EVENT-" + self.smart_data["event_name"], None)
        
    return "success"