
def execute(self, inputs, outputs, gvm):
    self.logger.info("Hello {}".format(self.name))
    grasp_plan = inputs["grasp_plan"]
    place_id = grasp_plan.to_workspace_id
    if place_id =="2" or place_id =="3":
        return "other_pallet"  
    elif place_id == "1":
        return "cache"              
    else:    
        return "success"
