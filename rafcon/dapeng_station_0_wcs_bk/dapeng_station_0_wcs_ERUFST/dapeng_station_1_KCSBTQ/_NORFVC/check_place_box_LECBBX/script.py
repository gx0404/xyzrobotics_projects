from xyz_motion import PlanningEnvironmentRos
from xyz_env_manager.client import get_planning_environment
from xyz_env_manager.client import clear_planned_items

def execute(self, inputs, outputs, gvm):
    self.logger.info("Hello {}".format(self.name))
    if not gvm.get_variable("motion_payload", per_reference=True, default=None):
        planning_env_msg = get_planning_environment()
        planning_env = PlanningEnvironmentRos.from_ros_msg(planning_env_msg)
    else:
        last_payload = gvm.get_variable("motion_payload", per_reference=True, default=None)
        planning_env = PlanningEnvironmentRos.from_ros_msg(last_payload["planning_environment"])
    row = gvm.get_variable("row", per_reference=False, default=None)
    place_id = inputs["place_id"]
    if place_id=="1":
        place_container_items = planning_env.get_container_items("1")    
        if len(place_container_items)%row==0 and len(place_container_items)//row==1:
            gvm.set_variable("motion_payload", None, per_reference=True)
            return "place_camera"
        else:
            return "success"
    else:    
        return "success"
