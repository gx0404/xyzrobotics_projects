
def execute(self, inputs, outputs, gvm):
    self.logger.info("Hello {}".format(self.name))
    move_camera_flag = gvm.get_variable("move_camera_flag", per_reference=False, default=None)
    plan_path = gvm.get_variable("plan_path", per_reference=False, default=None)
    if move_camera_flag and plan_path:
        self.logger.info(f"存在搜索路径,并且需要拍照")
        gvm.set_variable("move_camera_flag", None, per_reference=False)   
        return  "success"    
        return  "move_camera"   
    elif move_camera_flag and not plan_path: 
        self.logger.info(f"不存在搜索路径,并且需要拍照")   
        return  "success"          
    else:    
        return "success"
