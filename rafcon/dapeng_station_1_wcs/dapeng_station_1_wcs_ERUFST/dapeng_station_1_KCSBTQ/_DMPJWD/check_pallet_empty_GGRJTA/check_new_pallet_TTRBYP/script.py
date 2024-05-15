from xyz_env_manager.client import get_container_items
from xyz_env_manager.client import get_unfinished_planned_items
def execute(self, inputs, outputs, gvm):
    self.logger.info("Hello {}".format(self.name))
    init_place_list = ["0","1"]
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
