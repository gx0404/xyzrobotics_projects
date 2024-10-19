
def execute(self, inputs, outputs, gvm):
    self.logger.info("Hello {}".format(self.name))
    plan_path = gvm.get_variable("merge_1", per_reference=False, default=None)
    if not plan_path:
        return "pick_complete"
    else:
        return "success"
