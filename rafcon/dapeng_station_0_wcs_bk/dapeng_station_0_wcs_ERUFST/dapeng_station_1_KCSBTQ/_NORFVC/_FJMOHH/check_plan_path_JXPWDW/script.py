
def execute(self, inputs, outputs, gvm):
    self.logger.info("Hello {}".format(self.name))
    plan_path = gvm.get_variable("restore_depal_path", per_reference=False, default=None)
    if not plan_path:
        return "pick_complete"
    else:
        return "success"
