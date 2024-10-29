
def execute(self, inputs, outputs, gvm):
    self.logger.info("Hello {}".format(self.name))
    gvm.set_variable("motion_payload", None, per_reference=True)
    pick_tote_data = gvm.get_variable("pick_tote_data", per_reference=False, default=None)
    plan_path = gvm.get_variable("plan_path", per_reference=False, default=None)
    if not plan_path and not pick_tote_data:
        return "pick_complete"
    else:
        return "success"
