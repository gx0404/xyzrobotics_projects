
def execute(self, inputs, outputs, gvm):
    self.logger.info("Hello {}".format(self.name))
    outputs["pose_1"] = self.smart_data["pose_1"]
    outputs["pose_2"] = self.smart_data["pose_2"]
    outputs["pose_3"] = self.smart_data["pose_3"]
    outputs["pose_4"] = self.smart_data["pose_4"]
    return "success"
