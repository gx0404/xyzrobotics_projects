
def execute(self, inputs, outputs, gvm):
    self.logger.info("Hello {}".format(self.name))
    grasp_plan = inputs["grasp_plan"]
    place_id = grasp_plan.to_workspace_id
    if place_id =="2" or place_id =="3":
        self.logger.info(f"笼车任务")
        tf_flange_object = inputs["tf_flange_object"]
        tf_flange_object = list(map(lambda x:round(x,2),tf_flange_object))
        tf_flange_object_rotation = tf_flange_object[3:7]
        if tf_flange_object_rotation == [0, 1, -0.0, -0.0]\
            or tf_flange_object_rotation == [0, -1, -0.0, -0.0]:
            self.logger.info("条码朝向选择1")
            return "other_pallet_1"
        elif tf_flange_object_rotation == [1.0, 0.0, 0.0, -0.0] \
            or tf_flange_object_rotation == [-1.0, 0.0, 0.0, -0.0]:
            self.logger.info("条码朝向选择2")
            return "other_pallet_2" 
        else:
            raise "奇怪的tf_flange_obj"           
    else:    
        return "success"
