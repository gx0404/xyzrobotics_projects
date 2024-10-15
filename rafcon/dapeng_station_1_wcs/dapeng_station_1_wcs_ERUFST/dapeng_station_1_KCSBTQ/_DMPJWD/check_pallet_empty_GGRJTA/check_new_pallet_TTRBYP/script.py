from xyz_env_manager.client import get_container_items
from xyz_env_manager.client import get_unfinished_planned_items
def execute(self, inputs, outputs, gvm):
    self.logger.info("Hello {}".format(self.name))
    sku_info = inputs["sku_info"] 
    sku_dimension = [sku_info["length"],sku_info["width"],sku_info["height"]]
    sku_dimension = list(map(lambda x:round(x,2),sku_dimension))    
    if sku_dimension==[0.4,0.3,0.23]:
        self.logger.info(f"判断为中欧")
        init_place_list = ["4","5"]
    elif sku_dimension==[0.6,0.4,0.23]:
        self.logger.info(f"判断为大欧")
        init_place_list = ["0","1"]
    else:
        raise "无效的尺寸"    
                
    check_place_list = []
    for place_id in init_place_list:
        unfinished_items = get_unfinished_planned_items(place_id)
        if not unfinished_items:
            check_place_list.append(place_id)
            
    self.logger.info(f"需要检查的托盘号：{check_place_list}")
    
    if check_place_list:
        gvm.set_variable("check_place_list", check_place_list, per_reference=False)  
        return "check_place"
    else:
        self.logger.info(f"所有托盘都拍照确认为空过")    
        return "success"
