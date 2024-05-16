
def execute(self, inputs, outputs, gvm):
    self.logger.info("Hello {}".format(self.name))
    clamp_name_io = self.smart_data["clamp_name_io"]
    pick_un_clamp_collision_name = inputs["place_un_clamp_collision_name"]
    if len(pick_un_clamp_collision_name)==3:
        outputs["second_up_down_port_list"] = [5, 6, 7, 8, 9, 10]    
        outputs["second_up_down_value_list_down"] = [1, 0, 1, 0, 1, 0]
        return "no_clamp"
    up_down_port_list = []
    up_down_value_list_down = []
    up_down_value_list_up = []
    
    close_open_port_list = []
    close_open_value_list = []
    
    second_up_down_port_list = []
    second_up_down_value_list_down = []
    second_up_down_value_list_up = []
    
    for clamp_name in clamp_name_io.keys():
        if clamp_name not in pick_un_clamp_collision_name:
            self.logger.info(f"第一次缩回气缸{clamp_name}")
            up_down_port_list+=clamp_name_io[clamp_name]["up_down"]
            up_down_value_list_down+=clamp_name_io[clamp_name]["up_down_value"]
            up_down_value_list_up+=list(map(lambda x:1-x,clamp_name_io[clamp_name]["up_down_value"]))
            
            
            self.logger.info(f"第一次打开气缸{clamp_name}")
            close_open_port_list+=clamp_name_io[clamp_name]["close_open"]            
            close_open_value_list+=clamp_name_io[clamp_name]["close_open_value"]
        else:
            
            self.logger.info(f"第二次缩回气缸{clamp_name}")
            second_up_down_port_list+=clamp_name_io[clamp_name]["up_down"]
            second_up_down_value_list_down+=clamp_name_io[clamp_name]["up_down_value"]
            second_up_down_value_list_up+=list(map(lambda x:1-x,clamp_name_io[clamp_name]["up_down_value"]))            
    
    self.logger.info(f"第一次缩回气缸{up_down_port_list}") 
    self.logger.info(f"第一次打开气缸{close_open_port_list}")
    self.logger.info(f"第二次缩回气缸{second_up_down_port_list}") 
    
    
    
    
    outputs["up_down_port_list"] = up_down_port_list    
    outputs["up_down_value_list_down"] = up_down_value_list_down  
    outputs["up_down_value_list_up"] = up_down_value_list_up
    
    outputs["close_open_port_list"] = close_open_port_list    
    outputs["close_open_value_list"] = close_open_value_list  
        
    outputs["second_up_down_port_list"] = second_up_down_port_list    
    outputs["second_up_down_value_list_down"] = second_up_down_value_list_down
    outputs["second_up_down_value_list_up"] = second_up_down_value_list_up       
        
    return "success"
