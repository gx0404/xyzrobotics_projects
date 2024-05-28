
def execute(self, inputs, outputs, gvm):
    self.logger.info("Hello {}".format(self.name))
    place_id = inputs["place_id"]
    camera_pose_dict = self.smart_data["camera_pose_dict"]
    camera_pose = camera_pose_dict[place_id]
    outputs["camera_pose"] = camera_pose
    z_angle = False       
    outputs["z_angle"] = z_angle   
    if place_id == "0":
        scan_code = 4
    else:
        scan_code = 11
    outputs["scan_code"] = scan_code               
    return "success"
