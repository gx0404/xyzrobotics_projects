
def execute(self, inputs, outputs, gvm):
    self.logger.info("Hello {}".format(self.name))
    grasp_plan = inputs["grasp_plan"]
    place_id = str(grasp_plan.to_workspace_id)
    row = gvm.get_variable("row", per_reference=False, default=None)   
    self.logger.info(f"row is {row}") 
    self.logger.info(f"place_id is {place_id}")
    if row==5:   
        if place_id == "0":
            outputs["place_drop_buffer"] = 0.018
        else:
            outputs["place_drop_buffer"] = 0.02   
    else:
        if place_id == "4":
            outputs["place_drop_buffer"] = -0.007
        else:
            outputs["place_drop_buffer"] = -0.007                        
    return "success"
