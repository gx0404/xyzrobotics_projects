
def execute(self, inputs, outputs, gvm):
    self.logger.info("Hello {}".format(self.name))
    place_id = inputs["place_id"]
    camera_pose_dict = self.smart_data["camera_pose_dict"]
    camera_pose = camera_pose_dict[place_id]
    outputs["camera_pose"] = camera_pose
    if place_id in ["1","3","0","2"]:
        z_angle = False
    else:
        z_angle = True
    outputs["z_angle"] = z_angle 
    if place_id in ["4","0","3","2"]:
        scan_code = 8
    elif place_id in ["5"]:
        scan_code = 9        
    else:
        scan_code = 7
    
    outputs["scan_code"] = scan_code    
                      
    return "success"
