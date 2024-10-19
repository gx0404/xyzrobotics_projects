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
    #输出给后端此次抓取数据
    pick_data  = {pick_box_id:barcode}
    outputs["pick_data"] = pick_data

    #更新拣配托盘数据
    place_box_id = "1"

    pallet_tote_data[place_box_id] = {
        "barcode":barcode
    }
    #输出给后端此次放置数据
    place_data = {place_box_id:barcode}

    #获取缓存区所有箱子
    pick_container_items = planning_env.get_container_items(pick_id)
    #缓存区数据更新
    #校验数据
    if len(pick_container_items)!=len(cache_pallet_tote_data)-1:
        self.logger.info(f"缓存区数据与当前环境数量长度不一致")
        raise f"缓存区数据与当前环境数量长度不一致"  
    from_pick_id = cache_pallet_tote_data[pick_box_id]["from_pick_id"] 
    cache_pallet_tote_data.pop(pick_box_id)  
    outputs["pick_box_id"] = pick_box_id
    # for key,items in path.items():
    #     if place_box_id in items["pick_path"]:
    #         items["pick_path"].remove(place_box_id)  
    #         path["path_complete"].append(place_box_id) 
    #         self.logger.info(f"路径点{place_box_id}删除")
    #         if len(items["pick_path"])==1:
    #             if str(int(key)+1) in path.keys():
    #                 path.pop(key) 
    #                 path[str(int(key)+1)]["pick_path"].append(items["pick_path"][0])    
    #         else:
    #             pass        
    #         break   

    outputs["path"] = path
    outputs["cache_pallet_tote_data"] = cache_pallet_tote_data

    gvm.set_variable("pallet_tote_data", pallet_tote_data, per_reference=False)
    gvm.set_variable("pick_tote_data", pick_tote_data, per_reference=False)   
    return "success"

