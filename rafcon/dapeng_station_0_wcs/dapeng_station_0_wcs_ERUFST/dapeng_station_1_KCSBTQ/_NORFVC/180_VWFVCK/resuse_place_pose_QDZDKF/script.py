
def execute(self, inputs, outputs, gvm):
    self.logger.info("Hello {}".format(self.name))
    object_poses = gvm.get_variable("object_poses", per_reference=False, default=None)
    if not object_poses:
        return "fail"
    else:    
        return "success"
