import rospy
from xyz_motion import RobotDriver

def execute(self, inputs, outputs, gvm):
    """ 
    Set robot's digit output.

    Args:
        Inputs Data:
            None

        Outputs Data:
            None

        Gvm: 
            None
            
    Properties Data:
        comment, (unicode): Default value (u"set_robot_input").
            The comment of this state. This will show in state's GUI block.
    
        robot_id, (unicode): Default value (u"0").
            Which robot you want to control.

        port_id, (int): Default value (0).
            Which port you want to control.

    Outcomes:
        0: success
        -1: aborted
        -2: preempted
    """
    self.logger.info("Running {}({})({})".format(self.name, self.unique_id, self.smart_data["comment"]))
    robot_id = int(str(self.smart_data["robot_id"]))
    rob_driver = RobotDriver(robot_id)
    port_ids = self.smart_data["port_ids"]
    values = self.smart_data["values"]
    if not len(port_ids):
        return "pass"
    assert len(port_ids) == len(values)
    list(map(rob_driver.set_digital_output, self.smart_data["port_ids"], self.smart_data["values"]))
    
    return "success"