
def execute(self, inputs, outputs, gvm):
    self.logger.info("Hello {}".format(self.name))
    pick_tote_data = inputs["pick_tote_data"]
    plan_path = gvm.get_variable("merge_pick_path", per_reference=False, default=None)
    if not plan_path:
        return "pick_complete"
    else:
        return "success"
