
def execute(self, inputs, outputs, gvm):
    self.logger.info("Hello {}".format(self.name))
    pick_id = inputs["pick_id"]
    camera_pose_dict = self.smart_data["camera_pose_dict"]
    camera_pose = camera_pose_dict[pick_id]
    outputs["camera_pose"] = camera_pose         
    return "success"
