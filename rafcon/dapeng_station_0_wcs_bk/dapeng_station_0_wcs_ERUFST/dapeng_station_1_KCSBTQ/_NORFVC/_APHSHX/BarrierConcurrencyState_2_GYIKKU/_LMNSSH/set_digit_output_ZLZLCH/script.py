from xyz_io_client.io_client import set_digit_output

def execute(self, inputs, outputs, gvm):
    """ 
    Set digit output to IO server. 

    This state depends on the ros node(IO server) which managers all IO devices(except for Robot's IO).
    (Intall IO server by ``sudo xyz_apt update``, ``sudo xyz_apt install xyz-io``. Run by ``rosrun xyz_io io_server.py -d config_file``.)

    Args:
        Inputs Data:
            None

        Outputs Data:
            None

        Gvm: 
            None
            
    Properties Data:
        comment, (unicode): Default value (u"set_digit_output").
            The comment of this state. This will show in state's GUI block.
        
        device_id, (unicode):  Default value (u"1").

        port, (int): Default value (0).
            The port of setting.
        
        value, (bool): Default value (True).
            The value want to set the io device's digit output.

    Outcomes:
        0: success
        -1: aborted
        -2: preempted
    """
    self.logger.info("Running {}({})({})".format(self.name, self.unique_id, self.smart_data["comment"]))
    set_digit_output(self.smart_data["device_id"], self.smart_data["port"], self.smart_data["value"])
    return "success"
