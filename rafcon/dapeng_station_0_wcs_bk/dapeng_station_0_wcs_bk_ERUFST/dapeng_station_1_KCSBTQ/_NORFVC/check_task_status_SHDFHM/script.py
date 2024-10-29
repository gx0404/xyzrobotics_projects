
def execute(self, inputs, outputs, gvm):
    self.logger.info("Hello {}".format(self.name))
    task_info = inputs["task_info"]
    task_status = task_info["task_status"]
    if task_status==1 or task_status==0:
        return "depal"
    elif task_status==11:
        return "restore"
    elif task_status==12:
        return "restore_180"
    else:
        raise "无效的任务状态"    
