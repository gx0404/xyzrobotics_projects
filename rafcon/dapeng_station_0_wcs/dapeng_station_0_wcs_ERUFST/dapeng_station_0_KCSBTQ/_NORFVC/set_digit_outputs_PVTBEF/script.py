from xyz_io_client.io_client import set_digit_outputs, set_digit_output

def execute(self, inputs, outputs, gvm):
    """ 
    Set digit outputs to IO server. 

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
        comment, (unicode): Default value (u"set_digit_outputs").
            The comment of this state. This will show in state's GUI block.
        
        device_id, (unicode):  Default value (u"1").

        ports, (list): Default value ([0]).
            The ports that we want to set these ports

        value_list, (list): Default value ([0]).
            The value list that want to set the io device's digits output. These is one-to-one correspond to ports.


    Raises:
        Exception: invalid input

    Outcomes:
        0: success
        -1: aborted
        -2: preempted
    """
    ## TODO For support Tip's IO list, we use for loop to set_digit_input.
    ## We can use set_digit_inputs to accelerate the communication.

    self.logger.info("Running {}({})({})".format(self.name, self.unique_id, self.smart_data["comment"]))
    assert len(self.smart_data["ports"]) == len(self.smart_data["value_list"])
    for value in self.smart_data["value_list"]:
        if not isinstance(value, int):
            raise Exception("The value in value_list must interger")

    for port, value in zip(self.smart_data["ports"], self.smart_data["value_list"]):
        set_digit_output(self.smart_data["device_id"], port, value)

    return "success"
