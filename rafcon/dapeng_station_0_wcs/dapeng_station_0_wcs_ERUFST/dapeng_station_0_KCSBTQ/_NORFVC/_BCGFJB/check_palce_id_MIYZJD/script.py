
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
                outputs["scan_joints"] = [[-0.912896393,-1.329641889,1.43652780,-2.2192065],
                                          [-0.912896393,-1.329641889,1.43652780,-2.2192065+3.1415926*2]]
            elif tf_flange_object_rotation == [1.0, 0.0, 0.0, -0.0] \
                or tf_flange_object_rotation == [-1.0, 0.0, 0.0, -0.0]:                   
                self.logger.info("条码朝向选择2")
                outputs["scan_joints"] = [[-1.00481772,-1.2004323,1.264149,0.8304647],]
                                          #[-0.970460,-1.157250,1.2110,1.01568-3.1415926*2]]                
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
                outputs["scan_joints"] = [[-0.933790197722276,-0.7404413372733902,0.5088852357967713,-0.4600587077125722],
                                          [-0.933790197722276,-0.7404413372733902,0.5088852357967713,-0.4600587077125722+3.1415926*2]]
            elif tf_flange_object_rotation == [-0.707, -0.707, -0.0, 0.0] \
                or tf_flange_object_rotation == [0.707, 0.707, -0.0, 0.0]:
                self.logger.info("条码朝向选择2")
                outputs["scan_joints"] = [[-0.9788881393470503,-0.9967947636869067,0.9500012996170473,2.6364360042524475],
                                          [-0.9788881393470503,-0.9967947636869067,0.9500012996170473,2.6364360042524475-3.1415926*2]]
            else:
                raise "奇怪的tf_flange_obj"    
            return "cage_pallet"  
        else:
            raise "奇怪的sku_dimension"
    else:    
        return "success"
