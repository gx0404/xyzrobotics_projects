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
    out_port_ids = [15, 16, 19, 20]
    out_values = []
    if 3 in port_ids:
        #夹紧时阻挡气缸闭合
        if not values[port_ids.index(3)]:
            rob_driver.set_digital_output(11,1) 
            rob_driver.set_digital_output(17,0) 
            out_values+=[1,0]   
    else:
        #阻挡气缸伸出
        rob_driver.set_digital_output(11,0)   
        rob_driver.set_digital_output(17,1)   
        out_values+=[0,1]                      


    if 4 in port_ids:
        #夹紧时阻挡气缸闭合
        if not values[port_ids.index(4)]:
            rob_driver.set_digital_output(12,1)
            rob_driver.set_digital_output(18,0) 
            out_values+=[1,0]    
    else:
        #阻挡气缸伸出
        rob_driver.set_digital_output(12,0)
        rob_driver.set_digital_output(18,1)  
        out_values+=[0,1]                           
                
                     
    assert len(port_ids) == len(values)
    list(map(rob_driver.set_digital_output, self.smart_data["port_ids"], self.smart_data["values"]))
    outputs["out_port_ids"] = out_port_ids
    outputs["out_values"] = out_values
    return "success"