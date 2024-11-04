
def execute(self, inputs, outputs, gvm):
    self.logger.info("Hello {}".format(self.name))
    grasp_plan = inputs["grasp_plan"]
    place_id = grasp_plan.to_workspace_id
    sku_dimension = grasp_plan.objects[0].primitives[0].dimensions
    sku_dimension = list(map(lambda x:round(x,2),sku_dimension))
    
    if place_id =="2" or place_id =="3":
        if sku_dimension==[0.6,0.4,0.23]:
            self.logger.info(f"笼车大欧任务")
            tf_flange_object = inputs["tf_flange_object"]
            tf_flange_object = list(map(lambda x:round(x,3),tf_flange_object))
            tf_flange_object_rotation = tf_flange_object[3:7]
            if tf_flange_object_rotation == [0, 1, -0.0, -0.0]\
                or tf_flange_object_rotation == [0, -1, -0.0, -0.0]:
                # outputs["scan_joints"] = [[-0.82800,-1.28698,1.374180,-1.983461],
                #                           [-0.82800,-1.28698,1.374180,-1.983461+3.1415926*2]]                    
                self.logger.info("条码朝向选择1")              
                outputs["scan_joints"] = [[-0.837768,-1.29445009,1.39976605,-2.066069257],
                                          [-0.837768,-1.29445009,1.39976605,-2.066069257*2]]
            elif tf_flange_object_rotation == [1.0, 0.0, 0.0, -0.0] \
                or tf_flange_object_rotation == [-1.0, 0.0, 0.0, -0.0]:
                outputs["scan_joints"] = [[-0.93697444,-1.190971893,1.2592658763,0.9763177],]
                                          #[-0.970460,-1.157250,1.2110,1.01568-3.1415926*2]]                       
                self.logger.info("条码朝向选择2")               
            else:
                raise "奇怪的tf_flange_obj"
            return "cage_pallet"
            
        elif sku_dimension==[0.4,0.3,0.23]:
            self.logger.info(f"笼车中欧任务")
            tf_flange_object = inputs["tf_flange_object"]
            tf_flange_object = list(map(lambda x:round(x,3),tf_flange_object))
            tf_flange_object_rotation = tf_flange_object[3:7]
            if tf_flange_object_rotation == [-0.707, 0.707, -0.0, 0.0]\
                or tf_flange_object_rotation == [0.707, -0.707, -0.0, 0.0]:
                self.logger.info("条码朝向选择1")
                outputs["scan_joints"] = [[-0.9152269,-1.096110,1.136479,-0.59232903],
                                          [-0.9152269,-1.096110,1.136479,-0.59232903+3.1415926*2]]
            elif tf_flange_object_rotation == [-0.707, -0.707, -0.0, 0.0] \
                or tf_flange_object_rotation == [0.707, 0.707, -0.0, 0.0]:
                self.logger.info("条码朝向选择2")
                outputs["scan_joints"] = [[-0.97987557,-1.2608772,1.371084702,2.484615032],
                                          [-0.97987557,-1.2608772,1.371084702,2.484615032-3.1415926*2]]
            else:
                raise "奇怪的tf_flange_obj"    
            return "cage_pallet"  
        else:
            raise "奇怪的sku_dimension"
    else:    
        return "success"
