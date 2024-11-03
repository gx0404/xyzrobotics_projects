
def execute(self, inputs, outputs, gvm):
    self.logger.info("Hello {}".format(self.name))
    grasp_plan = inputs["grasp_plan"]
    place_id = str(grasp_plan.to_workspace_id)
    if place_id == "0":
        outputs["place_drop_buffer"] = 0.015
    else:
        outputs["place_drop_buffer"] = 0.015        
    return "success"
