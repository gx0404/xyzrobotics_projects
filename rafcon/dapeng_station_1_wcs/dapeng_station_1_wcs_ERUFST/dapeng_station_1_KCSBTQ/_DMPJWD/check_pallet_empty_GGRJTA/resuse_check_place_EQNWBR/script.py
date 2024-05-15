
def execute(self, inputs, outputs, gvm):
    self.logger.info("Hello {}".format(self.name))
    check_place_list = gvm.get_variable("check_place_list", per_reference=False, default=None)
     
    if not check_place_list:
        gvm.set_variable("check_place_list", None, per_reference=False)   
        return "empty"
    else:    
        place_id = check_place_list[0]
        self.logger.info(f"检测放置空间{place_id}是否为空")
        outputs["place_id"] = place_id
        check_place_list.pop(0)
        gvm.set_variable("check_place_list", check_place_list, per_reference=False)  
        return "success"