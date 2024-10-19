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
    self.logger.info(f"当前位置{place_id}存在实际料箱{len(container_items)}个，规划箱子{len(unfinished_items)}个")
    if container_items and not unfinished_items:
        self.logger.info(f"笼车码垛无规划箱子,已码满")
        outputs["is_pal_pallet_full"] = True
        outputs["place_id"] = place_id
        return "full"
    else:
        outputs["is_pal_pallet_full"] = False        
        return "success"
