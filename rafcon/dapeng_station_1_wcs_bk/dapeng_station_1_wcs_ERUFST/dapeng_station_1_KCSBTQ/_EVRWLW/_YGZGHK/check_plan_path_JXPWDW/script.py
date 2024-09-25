
def execute(self, inputs, outputs, gvm):
    self.logger.info("Hello {}".format(self.name))
    plan_path = gvm.get_variable("merge_2", per_reference=False, default=None)
    if not plan_path:
        gvm.set_variable("motion_payload", None, per_reference=True) 
        return "pick_complete"
    else:
        return "success"
