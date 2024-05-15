from xyz_motion import SE3, pose_to_list

def execute(self, inputs, outputs, gvm):
    self.logger.info("Hello {}".format(self.name))
    update_tf_map_flange_list = gvm.get_variable("tf_map_flange_list", per_reference=False, default=None)
    #添加放置到全局变量
    tf_base_place_obj = SE3(inputs["object_poses"][0])
    tf_base_place_obj_180 = tf_base_place_obj*SE3([0,0,0,0,0,1,0])
    gvm.set_variable("object_poses", [tf_base_place_obj_180.xyz_quat], per_reference=False)  
      
    if not update_tf_map_flange_list:
        return "fail"
    else:    
        return "success"
