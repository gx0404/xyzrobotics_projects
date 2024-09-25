import requests,json
from xyz_env_manager.client import get_planning_environment
from xyz_motion import PlanningEnvironmentRos,pose_to_list

def execute(self, inputs, outputs, gvm):

    planning_env_msg = get_planning_environment()
    planning_env = PlanningEnvironmentRos.from_ros_msg(planning_env_msg)

    self.logger.info("Hello {}".format(self.name))
    self.logger.info(f"get_pallet_data")
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
    #获取抓取箱子的位置
    pick_box_pose = pose_to_list(item.origin)
    #输出给后端此次抓取数据
    pick_data  = {pick_box_id:barcode}
    outputs["pick_data"] = pick_data
    self.logger.info(f"pick_data is {pick_data}")

    #更新回溯路径
    path["path_complete"] = []
    path_key = str(len(pick_tote_data))    
    if path_key not in path:
        path[path_key] = {}
        path[path_key]["direction"] = []
        path[path_key]["pick_path"] = []
    path[path_key]["direction"].append(current_direction) 
    path[path_key]["pick_path"].append(pick_box_id)
    if pick_box_id in pick_tote_data.keys(): 
        path[path_key]["target_box"] = pick_box_id   
        
    #添加原先抓取箱子在拣配托盘上的姿态    
    if "from_pick_pose_dict" not in path.keys():
        path["from_pick_pose_dict"] = {}
    else:
        path["from_pick_pose_dict"][pick_box_id] = pick_box_pose
    outputs["path"] = path
               
    if place_id=="1":
        #获取缓存区所有箱子
        place_container_items = planning_env.get_container_items(place_id)
        #获取此次抓取的位置号
        place_box_id_dict = inputs["place_box_id"]
        for key,vaules in place_box_id_dict.items():
            place_box_id = key    
        #输出给后端此次放置数据
        place_data = {place_box_id:barcode}
        

        #缓存区数据更新
        #校验数据
        if place_box_id in cache_pallet_tote_data.keys():
            self.logger.info(f"此次放置位置号:{place_box_id}与当前缓存区已有位置号重复")    
            raise f"此次放置位置号:{place_box_id}与当前缓存区已有位置号重复"  
        if len(place_container_items)!=len(cache_pallet_tote_data)+1:
            self.logger.info(f"缓存区数据与当前环境数量长度不一致")
            raise f"缓存区数据与当前环境数量长度不一致"
        #找到此次放置的箱子位置信息
        place_items_ids = grasp_plan.objects[0].name
        current_place_item = list(filter(lambda x:x.name==place_items_ids,place_container_items))[0]
        cache_pallet_tote_data[place_box_id] = {
            "barcode":barcode,
            "pose_rotation":pose_to_list(current_place_item.origin)[3:7],
            "from_pick_id":pick_box_id
        }
        #更新拣配托盘数据
        pallet_tote_data.pop(pick_box_id)
        #更新目标箱子数据
        if pick_box_id in pick_tote_data.keys(): 
            pick_tote_data.pop(pick_box_id)
            outputs["pick_tote_data"] = pick_tote_data

        outputs["pallet_tote_data"] = pallet_tote_data
        outputs["cache_pallet_tote_data"] = cache_pallet_tote_data
        outputs["place_data"] = place_data
        self.logger.info(f"place data is {place_data}")
    else:
        #输出给后端此次放置数据
        place_data = {1:barcode}
        #更新拣配托盘数据
        pallet_tote_data.pop(pick_box_id)
        #更新目标箱子数据
        if pick_box_id in pick_tote_data.keys(): 
            pick_tote_data.pop(pick_box_id)
            outputs["pick_tote_data"] = pick_tote_data  
        outputs["pallet_tote_data"] = pallet_tote_data       
        outputs["place_data"] = place_data 
        self.logger.info(f"place data is {place_data}")
 
            
    return "success"
