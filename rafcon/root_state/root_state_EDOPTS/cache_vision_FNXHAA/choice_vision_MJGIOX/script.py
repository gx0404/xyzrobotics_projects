
def execute(self, inputs, outputs, gvm):
    self.logger.info("Hello {}".format(self.name))
    cache_pallet_tote_data = inputs["cache_pallet_tote_data"]
    box_id_list = cache_pallet_tote_data.keys()
    row = gvm.get_variable("row", per_reference=False, default=None)
    if row==5:
        self.logger.info("row is 5")
        check_box_id_list = [str(i) for i in range(1, 6)]
        check_flag = False
        for box_id in box_id_list:
            if box_id in check_box_id_list:
                continue
            else:
                self.logger.info(f"box_id not in {check_box_id_list}")
                check_flag = True   
                break
        if not check_flag:    
            return "other_vision"  
        else:                      
            return "success" 
    elif row==9:
        self.logger.info("row is 9")
        check_box_id_list = [str(i) for i in range(1, 10)]
        check_flag = False
        for box_id in box_id_list:
            if box_id in check_box_id_list:
                continue
            else:
                self.logger.info(f"box_id not in {check_box_id_list}")
                check_flag = True   
                break
        if not check_flag:    
            return "other_vision"  
        else:                      
            return "success"
    else:
        self.logger.info("row is none")
        return "success"     
