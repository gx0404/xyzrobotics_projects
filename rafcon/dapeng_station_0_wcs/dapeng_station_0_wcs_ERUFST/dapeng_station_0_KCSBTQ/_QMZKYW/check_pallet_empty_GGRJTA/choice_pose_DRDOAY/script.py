
def execute(self, inputs, outputs, gvm):
    self.logger.info("Hello {}".format(self.name))
    place_id = inputs["place_id"]
    camera_pose_dict = self.smart_data["camera_pose_dict"]
    camera_pose = camera_pose_dict[place_id]
    outputs["camera_pose"] = camera_pose
    return "success"
