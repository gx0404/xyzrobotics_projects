
def execute(self, inputs, outputs, gvm):
    self.logger.info("Hello {}".format(self.name))
    restore_depal = gvm.get_variable("restore_depal", per_reference=False, default=None)
    gvm.set_variable("restore_depal", False, per_reference=False)
    if restore_depal:
        return "restore_depal"
    else:    
        return "success"
