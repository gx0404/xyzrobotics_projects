from xyz_env_manager.client import get_planning_environment
from xyz_motion import PlanningEnvironmentRos
import requests
import json
def execute(self, inputs, outputs, gvm):
    self.logger.info("Hello {}".format(self.name))
    pl = get_planning_environment()
    planning_env = PlanningEnvironmentRos.from_ros_msg(pl)
    pick_id = inputs["pick_id"]
    
    pick_contain_items = planning_env.get_container_items(pick_id)
    self.logger.info(f"当前位置{pick_id}存在实际料箱{len(pick_contain_items)}个")
    
    layer = gvm.get_variable("layer"+str(pick_id), per_reference=False, default=None)
    self.logger.info(f"当前层数为{layer}层")
    
    if layer==1 and not pick_contain_items:
        self.logger.info(f"笼车已抓空,回报任务完成")
        return "empty"
    else:     
        return "success"
