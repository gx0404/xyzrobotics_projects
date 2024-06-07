
def execute(self, inputs, outputs, gvm):
    self.logger.info("Hello {}".format(self.name))
    move_camera_flag = gvm.get_variable("move_camera_flag", per_reference=False, default=None)
    if move_camera_flag:
        gvm.set_variable("move_camera_flag", None, per_reference=False)      
        return  "move_camera"   
    else:    
        return "success"
