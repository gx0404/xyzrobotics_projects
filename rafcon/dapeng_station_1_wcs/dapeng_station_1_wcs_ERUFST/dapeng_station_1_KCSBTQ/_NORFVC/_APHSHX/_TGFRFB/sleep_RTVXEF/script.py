
def execute(self, inputs, outputs, gvm):
    """ 
    Sleep for some time

    Args:
        Inputs Data:
            None

        Outputs Data:
            None

        Gvm: 
            None
            

    Properties Data:
        comment, (unicode): Default value (u"sleep").
            The comment of this state. This will show in state's GUI block.
        sleep_time, (float): Default value (1.0).
            The Specified sleep time

    Outcomes:
        0: success
        -1: aborted
        -2: preempted
    """
    self.logger.info("Running {}({})({})".format(self.name, self.unique_id, self.smart_data["comment"]))
    self.preemptive_wait(self.smart_data["sleep_time"])
    return "success"
