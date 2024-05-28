
def execute(self, inputs, outputs, gvm):
    self.logger.info("Hello {}".format(self.name))
    update_tf_map_flange_list = gvm.get_variable("tf_map_flange_list", per_reference=False, default=None)

    #添加放置到全局变量
    gvm.set_variable("object_poses", inputs["object_poses"], per_reference=False)         
    if not update_tf_map_flange_list:
        return "fail"
    else:    
        return "success"
