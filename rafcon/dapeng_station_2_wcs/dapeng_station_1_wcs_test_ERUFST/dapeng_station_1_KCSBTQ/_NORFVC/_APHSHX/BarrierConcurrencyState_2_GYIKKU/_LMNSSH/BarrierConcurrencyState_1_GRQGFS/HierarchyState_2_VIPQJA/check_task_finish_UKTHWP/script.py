
def execute(self, inputs, outputs, gvm):
    self.logger.info("Hello {}".format(self.name))
    task_finish_flag = gvm.get_variable("task_finish_flag", per_reference=False, default=None)
    barcode_error_flag = gvm.get_variable("barcode_error_flag", per_reference=False, default=None)
    if task_finish_flag:
        if barcode_error_flag:
            self.logger.info(f"最后一个任务扫码失败")
            outputs["error"] = 99
        else:
            outputs["error"] = 0        
        return "task_finish"
    elif barcode_error_flag:
        outputs["error"] = 99
        self.logger.info(f"扫码失败,结束任务")
        return "task_finish"
    else:
        outputs["error"] = 0
        return "success"
