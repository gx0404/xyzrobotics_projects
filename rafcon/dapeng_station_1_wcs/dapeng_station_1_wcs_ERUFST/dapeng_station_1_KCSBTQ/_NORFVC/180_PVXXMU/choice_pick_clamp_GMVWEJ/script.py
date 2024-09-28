
def execute(self, inputs, outputs, gvm):
    self.logger.info("Hello {}".format(self.name))
    clamp_name_io = self.smart_data["clamp_name_io"]
    pick_un_clamp_collision_name = inputs["pick_un_clamp_collision_name"]
    
    up_down_port_list = []
    up_down_value_list = []
    
    close_open_port_list = []
    close_open_value_list_close = []
    close_open_value_list_open = []
    
    
    second_up_down_port_list = []
    second_up_down_value_list = []
    
    for clamp_name in clamp_name_io.keys():
        if clamp_name in pick_un_clamp_collision_name:
            self.logger.info(f"一次伸出气缸{clamp_name}")
            up_down_port_list+=clamp_name_io[clamp_name]["up_down"]
            up_down_value_list+=clamp_name_io[clamp_name]["up_down_value"]
            
            
        else:

            self.logger.info(f"二次伸出气缸{clamp_name}")
            second_up_down_port_list+=clamp_name_io[clamp_name]["up_down"]
            second_up_down_value_list+=clamp_name_io[clamp_name]["up_down_value"]
            
            self.logger.info(f"第一次夹紧气缸{clamp_name}")
            close_open_port_list+=clamp_name_io[clamp_name]["close_open"]
            close_open_value_list_close+=clamp_name_io[clamp_name]["close_open_value"]
            close_open_value_list_open+=list(map(lambda x:1-x,clamp_name_io[clamp_name]["close_open_value"]))      
    
    self.logger.info(f"第一次伸出气缸{up_down_port_list}") 
    self.logger.info(f"第一次夹紧信号{close_open_port_list}")
    self.logger.info(f"第二次伸出气缸{second_up_down_port_list}") 
    
    outputs["up_down_port_list"] = up_down_port_list    
    outputs["up_down_value_list"] = up_down_value_list  
    
    outputs["close_open_port_list"] = close_open_port_list    
    outputs["close_open_value_list_close"] = close_open_value_list_close
    outputs["close_open_value_list_open"] = close_open_value_list_open  
        
    outputs["second_up_down_port_list"] = second_up_down_port_list    
    outputs["second_up_down_value_list"] = second_up_down_value_list         
        
    return "success"
