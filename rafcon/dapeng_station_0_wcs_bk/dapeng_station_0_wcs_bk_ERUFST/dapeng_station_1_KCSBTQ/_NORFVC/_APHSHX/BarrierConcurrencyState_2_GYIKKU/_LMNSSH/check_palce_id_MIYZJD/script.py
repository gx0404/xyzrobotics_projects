
def execute(self, inputs, outputs, gvm):
    self.logger.info("Hello {}".format(self.name))
    trajectory = inputs["trajectory"]
    place_id = trajectory["grasp_plan"].to_workspace_id
    if place_id =="2" or place_id =="3":
        self.logger.info(f"正在笼车,准备去扫码")
        outputs["acc"] = [10,10]
        outputs["speed"] = [50,50]
        return "other_pallet"
    else:   
        outputs["acc"] = [100,100]
        outputs["speed"] = [100,100]    
        return "success"
