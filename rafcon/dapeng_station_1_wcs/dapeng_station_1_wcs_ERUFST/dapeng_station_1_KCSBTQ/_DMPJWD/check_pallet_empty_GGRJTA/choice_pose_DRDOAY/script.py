
def execute(self, inputs, outputs, gvm):
    self.logger.info("Hello {}".format(self.name))
    place_id = inputs["place_id"]
    camera_pose_dict = self.smart_data["camera_pose_dict"]
    camera_pose = camera_pose_dict[place_id]
    outputs["camera_pose"] = camera_pose
    if place_id in ["0","1"]:
        z_angle = False 
    else:
        z_angle = True          
    outputs["z_angle"] = z_angle   
    if place_id == "0":
        scan_code = 4
    elif place_id == "1":
        scan_code = 11
    elif place_id=="4":
        scan_code = 8
    elif place_id=="5":
        scan_code = 9     
               
    outputs["scan_code"] = scan_code               
    return "success"
