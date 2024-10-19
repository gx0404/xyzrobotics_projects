import requests
from xyz_env_manager.client import get_planning_environment
from xyz_motion import PlanningEnvironmentRos,pose_to_list

def execute(self, inputs, outputs, gvm):

    planning_env_msg = get_planning_environment()
    planning_env = PlanningEnvironmentRos.from_ros_msg(planning_env_msg)

    url = "http://127.0.0.1:7002/api/rafcon/get_pallet_data"
    try: 
        response = requests.post(url).json()
    except requests.exceptions.ConnectionError as e:
        raise Exception("Connection Error: {}".format(e))
    except: 
        raise Exception("Unknown errors occured when requesting response from hmi-back.")   
    
    if response["error"]!=0:
        raise Exception("Error occured when requesting response from hmi-back.")
    
    current_direction = response["current_direction"]
    outputs["current_direction"] = current_direction
    self.logger.info(f"current_direction is {current_direction}")

    pallet_tote_data = response["pallet_tote_data"]
    outputs["pallet_tote_data"] = pallet_tote_data
    self.logger.info(f"pallet_tote_data is {pallet_tote_data}")

    pick_tote_data = response["pick_tote_data"]
    outputs["pick_tote_data"] = pick_tote_data
    self.logger.info(f"pick_tote_data is {pick_tote_data}")

    cache_pallet_tote_data = response["cache_pallet_tote_data"]   
    outputs["cache_pallet_tote_data"] = cache_pallet_tote_data
    self.logger.info(f"cache_pallet_tote_data is {cache_pallet_tote_data}")

    path = response["path"]   
    outputs["path"] = path
    self.logger.info(f"path is {path}")

    pallet_tote_data_2 = response["pallet_tote_data_2"]   
    outputs["pallet_tote_data_2"] = pallet_tote_data_2
    self.logger.info(f"pallet_tote_data_2 is {pallet_tote_data_2}")
    
    pallet_tote_data_3 = response["pallet_tote_data_3"]   
    outputs["pallet_tote_data_3"] = pallet_tote_data_3
    self.logger.info(f"pallet_tote_data_3 is {pallet_tote_data_3}")
    
    pallet_tote_data_7 = response["pallet_tote_data_7"]   
    outputs["pallet_tote_data_7"] = pallet_tote_data_7
    self.logger.info(f"pallet_tote_data_7 is {pallet_tote_data_7}")
    
    pallet_tote_data_8 = response["pallet_tote_data_8"]   
    outputs["pallet_tote_data_8"] = pallet_tote_data_8
    self.logger.info(f"pallet_tote_data_8 is {pallet_tote_data_8}")
 

    execution_payload = inputs["execution_payload"]
    grasp_plan = execution_payload["grasp_plan"]
    pick_id = grasp_plan.from_workspace_id
    place_id = grasp_plan.to_workspace_id
    outputs["pick_id"] = pick_id
    outputs["place_id"] = place_id

    item = grasp_plan.objects[0]
    #获取条码
    barcode = item.additional_info.values[-2]
    #获取抓取的位置号
    pick_box_id = item.additional_info.values[-3]
    if pick_box_id != inputs["pick_box_id"]:
        raise "different pick_box_id"
    to_ws = item.additional_info.values[-1]
    
    pallet_tote_data_7 = {
        "pick_box_id":pick_box_id,
        "barcode":barcode,
        "to_ws":to_ws
    }
    outputs["pallet_tote_data_7"] = pallet_tote_data_7
    
    #获取抓取箱子的位置
    pick_box_pose = pose_to_list(item.origin)
    if "from_pick_pose_dict" not in path.keys():
        path["from_pick_pose_dict"] = {}
    else:
        path["from_pick_pose_dict"][pick_box_id] = pick_box_pose
    outputs["path"] = path

    gvm.set_variable("pallet_tote_data", pallet_tote_data, per_reference=False)
    gvm.set_variable("pick_tote_data", pick_tote_data, per_reference=False)   
    return "success"

