
def execute(self, inputs, outputs, gvm):
    self.logger.info("Hello {}".format(self.name))
    cache_pallet_tote_data = inputs["cache_pallet_tote_data"]
    box_id_keys = cache_pallet_tote_data.keys()
    box_id_list = []
    
    for i in box_id_keys:
        box_id_list.append(int(i))    
    row = gvm.get_variable("row", per_reference=False, default=None)
    if row==5:
        self.logger.info("row is 5")
        if len(box_id_list)<6:
            self.logger.info(f"layer is 1")
            for i in box_id_list:
                if i not in [i for i in range(1,6)]:
                    self.logger.info(f"layer is 1 ,but box id is {box_id_list}")
                    return "success" 
            self.logger.info(f"layer is 1 ,同一层")    
            return "other_vision"
        
        if len(box_id_list)%5:
            self.logger.info(f"非整层")
            return "success"
        
        check_flag = False
        for i in range(0,len(box_id_list)-5+1,5):
            if box_id_list[i] + 4!=box_id_list[i+4]:
                check_flag = True
                break
            
        if not check_flag:    
            return "other_vision"  
        else:                      
            return "success"     
                

    elif row==9:
        self.logger.info("row is 9")
        if len(box_id_list)<10:
            self.logger.info(f"layer is 1")
            for i in box_id_list:
                if i not in [i for i in range(1,10)]:
                    self.logger.info(f"layer is 1 ,but box id is {box_id_list}")
                    return "success" 
            self.logger.info(f"layer is 1 ,同一层")    
            return "other_vision" 
        
        if len(box_id_list)%9:
            self.logger.info(f"非整层")
            return "success"
        
        check_flag = False
        for i in range(0,len(box_id_list)-9+1,9):
            if box_id_list[i] + 8!=box_id_list[i+8]:
                check_flag = True
                break
            
        if not check_flag:    
            return "other_vision"  
        else:                      
            return "success"                        
    else:
        self.logger.info("row is none")
        return "success"     
