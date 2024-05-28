
def execute(self, inputs, outputs, gvm):
    self.logger.info("Hello {}".format(self.name))
    first_calculate = gvm.get_variable("first_calculate", per_reference=False, default=None)
    if not first_calculate:   
        gvm.set_variable("first_calculate", True, per_reference=False) 
        return "success"
    else:
        raise "计算失败"        
