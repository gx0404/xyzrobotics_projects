
def execute(self, inputs, outputs, gvm):
    self.logger.info("Hello {}".format(self.name))
    merge_depal = gvm.get_variable("merge_depal", per_reference=False, default=None)
    gvm.set_variable("merge_depal", False, per_reference=False)
    if merge_depal:
        return "merge_depal"
    else:    
        return "success"
