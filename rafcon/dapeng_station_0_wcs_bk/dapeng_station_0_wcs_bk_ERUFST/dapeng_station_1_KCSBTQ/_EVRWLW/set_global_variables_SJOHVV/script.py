def execute(self, inputs, outputs, gvm):

    self.logger.info("Running {}({})".format(self.name, self.unique_id))
    self.logger.info(self.smart_data["comment"])
    variables = self.smart_data["variables"]

    if len(variables) == 0:
        raise Exception("没有需要设置的全局变量")
    else:
        for key, val in variables.items():
            self.logger.info(f"设置全局变量: {key}, 初始值: {val}, 数据类型: {type(val)}")
            gvm.set_variable(key, val)

    return "success"
