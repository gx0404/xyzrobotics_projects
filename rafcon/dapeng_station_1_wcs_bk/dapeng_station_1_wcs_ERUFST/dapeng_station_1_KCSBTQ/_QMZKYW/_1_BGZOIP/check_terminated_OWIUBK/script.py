
def execute(self, inputs, outputs, gvm):
    self.logger.info("Hello {}".format(self.name))
    cache_terminated = gvm.get_variable("cache_terminated", per_reference=False, default=None)
    gvm.set_variable("cache_terminated" ,False,per_reference=False)
    if cache_terminated:     
        return "cache_terminated"
    else:
        return "success"
