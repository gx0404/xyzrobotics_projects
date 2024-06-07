
def execute(self, inputs, outputs, gvm):
    self.logger.info("Hello {}".format(self.name))
    trajectory = inputs["trajectory"]
    place_id = trajectory["grasp_plan"].to_workspace_id
    if place_id =="6":
        self.logger.info(f"正在放置输送线,等待信号")
        return "wait"
    else:    
        return "success"
