
def execute(self, inputs, outputs, gvm):
    self.logger.info("Hello {}".format(self.name))
    if inputs["place_id"] == "2":
        outputs["joint"]= self.smart_data["pose_1"]
    elif inputs["place_id"] == "3":     
        outputs["joint"]= self.smart_data["pose_2"] 
    return "success"
