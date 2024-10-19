
def execute(self, inputs, outputs, gvm):
    self.logger.info("Hello {}".format(self.name))
    task_finish_flag = gvm.get_variable("task_finish_flag", per_reference=False, default=None)
    if task_finish_flag:
        outputs["error"] = 0        
        return "task_finish"
    else:
        outputs["error"] = 0
        return "success"
