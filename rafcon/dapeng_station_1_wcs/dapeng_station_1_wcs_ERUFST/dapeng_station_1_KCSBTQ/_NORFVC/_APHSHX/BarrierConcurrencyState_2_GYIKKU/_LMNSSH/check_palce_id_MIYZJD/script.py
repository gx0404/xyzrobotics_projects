
def execute(self, inputs, outputs, gvm):
    self.logger.info("Hello {}".format(self.name))
    trajectory = inputs["trajectory"]
    place_id = trajectory["grasp_plan"].to_workspace_id
    if place_id =="2" or place_id =="3":
        self.logger.info(f"正在笼车,准备去扫码")
        outputs["acc"] = [15,15]
        outputs["speed"] = [70,70]
        row = gvm.get_variable("row", per_reference=False, default=None)  
        if row == 5:
            outputs["pre_acc"] = [100,100]    
            outputs["pre_speed"] = [100,100] 
        else:
            outputs["pre_acc"] = [30,30]    
            outputs["pre_speed"] = [50,50]                 
        return "other_pallet"
    else:   
        lower_speed = gvm.get_variable("lower_speed", per_reference=False, default=None)  
        if lower_speed:
            outputs["pre_acc"] = [40,40]    
            outputs["pre_speed"] = [70,70]  
        else:  
            outputs["pre_acc"] = [100,100]    
            outputs["pre_speed"] = [100,100]                             
        outputs["acc"] = [100,100]
        outputs["speed"] = [100,100]    
        
        return "success"
