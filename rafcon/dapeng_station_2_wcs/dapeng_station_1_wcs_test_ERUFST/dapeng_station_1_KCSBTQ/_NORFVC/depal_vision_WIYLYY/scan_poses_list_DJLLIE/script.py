
def execute(self, inputs, outputs, gvm):
    self.logger.info("Hello {}".format(self.name))
    if not inputs["direction"]:
        direction = "C"
    else:
        direction = inputs["direction"]     
    if direction=="C":       
        outputs["pose_1"] = self.smart_data["pose_1"]
        outputs["pose_2"] = self.smart_data["pose_2"]
        outputs["pose_3"] = self.smart_data["pose_3"]
        outputs["pose_4"] = self.smart_data["pose_4"]
    else:
        outputs["pose_1"] = self.smart_data["pose_1_A"]
        outputs["pose_2"] = self.smart_data["pose_2_A"]
        outputs["pose_3"] = self.smart_data["pose_3_A"]
        outputs["pose_4"] = self.smart_data["pose_4_A"]            
    return "success"
