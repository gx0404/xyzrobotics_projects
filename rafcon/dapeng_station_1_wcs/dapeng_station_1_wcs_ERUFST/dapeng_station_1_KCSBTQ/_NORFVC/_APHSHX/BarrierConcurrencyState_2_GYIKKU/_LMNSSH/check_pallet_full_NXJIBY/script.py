from xyz_env_manager.client import get_planning_environment
from xyz_motion import PlanningEnvironmentRos
import requests
import json
def execute(self, inputs, outputs, gvm):
    self.logger.info("Hello {}".format(self.name))
    pl = get_planning_environment()
    planning_env = PlanningEnvironmentRos.from_ros_msg(pl)
    trajectory = inputs["trajectory"]
    place_id = trajectory["grasp_plan"].to_workspace_id  
    unfinished_items = planning_env.get_unfinished_planned_items(place_id)
    container_items = planning_env.get_container_items(place_id)
    
    sku_dimension = trajectory["grasp_plan"].objects[0].primitives[0].dimensions
    sku_dimension = list(map(lambda x:round(x,2),sku_dimension))
    
    #获取笼车中欧层数
    layer_num = gvm.get_variable("layer_num", per_reference=False, default=None)  
    self.logger.info(f"当前设置笼车中欧满层数为{layer_num}")
    self.logger.info(f"当前位置{place_id}存在实际料箱{len(container_items)}个，规划箱子{len(unfinished_items)}个")
    
    if place_id=="2" or place_id=="3":
        if len(container_items)==24 and sku_dimension==[0.6,0.4,0.23]:
            self.logger.info(f"笼车码垛无规划箱子,已码满")
            outputs["is_pal_pallet_full"] = True
            outputs["place_id"] = place_id
            return "full"
        elif len(container_items)==layer_num*8 and sku_dimension==[0.4,0.3,0.23]:
            self.logger.info(f"笼车码垛无规划箱子,已码满")
            outputs["is_pal_pallet_full"] = True
            outputs["place_id"] = place_id
            return "full"   
        else:
            outputs["is_pal_pallet_full"] = False        
            return "success"                     
    else:
        outputs["is_pal_pallet_full"] = False        
        return "success"
