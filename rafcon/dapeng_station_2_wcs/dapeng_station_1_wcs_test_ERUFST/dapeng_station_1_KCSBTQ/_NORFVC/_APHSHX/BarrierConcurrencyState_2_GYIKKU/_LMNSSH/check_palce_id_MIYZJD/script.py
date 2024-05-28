
def execute(self, inputs, outputs, gvm):
    self.logger.info("Hello {}".format(self.name))
    trajectory = inputs["trajectory"]
    place_id = trajectory["grasp_plan"].to_workspace_id
    if place_id =="2" or place_id =="3":
        self.logger.info(f"正在笼车,准备去扫码")
        return "other_pallet"
    else:    
        return "success"
