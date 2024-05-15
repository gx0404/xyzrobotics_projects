from xyz_env_manager.client import get_planning_environment
from xyz_motion import PlanningEnvironmentRos
import requests

def get_task_status():
    url = "http://127.0.0.1:7002/api/rafcon/get_multi_task_status"
    data = {
        }
    try:
        response = requests.post(url, json = data).json()
    except requests.exceptions.ConnectionError as e:
        raise Exception("Connection Error: {}".format(e))
    except: 
        raise Exception("Unknown errors occured when requesting response from hmi-back.")  
    task_status = response["task_status"]  
    return task_status

def undate_task_status(task_status):
    url = "http://127.0.0.1:7002/api/rafcon/update_multi_task_status"
    data = {
        "task_status":task_status
        }    
    try:
        response = requests.post(url, json = data).json()
    except requests.exceptions.ConnectionError as e:
        raise Exception("Connection Error: {}".format(e))
    except: 
        raise Exception("Unknown errors occured when requesting response from hmi-back.")  
    return True    

def execute(self, inputs, outputs, gvm):
    self.logger.info("Hello {}".format(self.name))
    pl = get_planning_environment()
    planning_env = PlanningEnvironmentRos.from_ros_msg(pl)
    place_id = inputs["place_id"]
    tote_type = inputs["tote_type"]
    unfinished_planned_items = planning_env.get_unfinished_planned_items(place_id)
        
    self.logger.info(f"放置空间{place_id}放置规划还剩{len(unfinished_planned_items)}个")
  
    if not unfinished_planned_items:
        container_items = planning_env.get_container_items(place_id)
        if not container_items:
            self.logger.info(f"放置空间无规划")
            return "place_plan_empty"
        else:
            self.logger.info(f"放置空间无规划,但存在料箱,可能是放满了")    
            return "full"
    else:
        unfinished_planned_items = sorted(unfinished_planned_items, key = lambda i: int((i.name).split("-")[1])) 
        place_item = unfinished_planned_items[0]
        box_dimension = list(place_item.primitives[0].dimensions)
        box_dimension = list(map(lambda x:round(x,3),box_dimension))
        self.logger.info(f"即将放置的料箱尺寸为{box_dimension}")
        task_status = get_task_status()
        if task_status==14:
            self.logger.info(f"当前任务状态为处理缓存区")
            #判断是否为中欧
            if box_dimension!=[0.4,0.3,0.23] or task_status==1:
                self.logger.info(f"已合并过规划,放置规划的尺寸不一致")
                raise f"已合并过规划,放置规划的尺寸不一致"
            plan_container_items = planning_env.get_container_items(place_id) 
            if place_id == "0":
                container_items = planning_env.get_container_items("7")  
                pallet_tote_data_7 = inputs["pallet_tote_data_7"]
                outputs["cache_id"] = "7"
                outputs["place_id"] = place_id
                if pallet_tote_data_7 and container_items:  
                    self.logger.info(f"还未抓取缓存区物料到托盘")    
                    return "depal_cache_0"
                elif not pallet_tote_data_7 and not container_items and plan_container_items:
                    self.logger.info(f"已抓取缓存区物料到托盘")  
                    return "depal_cache_1"
                else:
                    raise "缓存区数据和环境对不上"                        
            elif place_id == "1":
                container_items = planning_env.get_container_items("8")  
                pallet_tote_data_8 = inputs["pallet_tote_data_8"]
                outputs["cache_id"] = "8"
                outputs["place_id"] = place_id
                if pallet_tote_data_8 and container_items:  
                    self.logger.info(f"还未抓取缓存区物料到托盘")    
                    return "depal_cache_0"
                elif not pallet_tote_data_8 and not container_items and plan_container_items:
                    self.logger.info(f"已抓取缓存区物料到托盘")  
                    return "depal_cache_1"
                else:
                    raise "缓存区数据和环境对不上"    
            else:
                raise "无效的中欧放置工作空间"  
        else:    
            self.logger.info(f"当前任务状态正常")       
                                            
            if tote_type==0:
                self.logger.info(f"当前料箱为中欧")
                #判断是否为中欧
                if box_dimension!=[0.4,0.3,0.23]:
                    self.logger.info(f"已合并过规划,放置规划的尺寸不一致")
                    raise f"已合并过规划,放置规划的尺寸不一致"
                #只有两个托盘能放中欧，与缓存区一一对应
                if place_id == "0":
                    container_items = planning_env.get_container_items("7")    
                    pallet_tote_data_7 = inputs["pallet_tote_data_7"]
                    if pallet_tote_data_7 and container_items:                  
                        if pallet_tote_data_7["to_ws"]!=place_id:
                            raise "数据库缓存区7厂家放置空间号不一致"
                        self.logger.info(f"放置空间{place_id},缓存区有中欧,两个中欧依次放置")
                        outputs["cache_id"] = "7"
                        outputs["place_id"] = place_id
                        undate_task_status(14)
                        return "depal_cache_0"
                    elif (not pallet_tote_data_7) and (not container_items):
                        self.logger.info(f"放置空间{place_id},缓存区无中欧")   
                        outputs["cache_id"] = "7"   
                        outputs["place_id"] = place_id  
                        undate_task_status(13)            
                        return "pal_cache"    
                    else:
                        raise "缓存区数据和环境对不上"        
                elif place_id == "1":
                    container_items = planning_env.get_container_items("8")    
                    pallet_tote_data_8 = inputs["pallet_tote_data_8"]
                    if pallet_tote_data_8 and container_items:                  
                        if pallet_tote_data_8["to_ws"]!=place_id:
                            raise "数据库缓存区8厂家放置空间号不一致"
                        self.logger.info(f"放置空间{place_id},缓存区有中欧,两个中欧依次放置")
                        outputs["cache_id"] = "8"
                        outputs["place_id"] = place_id
                        undate_task_status(14)
                        return "depal_cache_0"
                    elif (not pallet_tote_data_8) and (not container_items):
                        self.logger.info(f"放置空间{place_id},缓存区无中欧")   
                        outputs["cache_id"] = "8"  
                        outputs["place_id"] = place_id  
                        undate_task_status(13)             
                        return "pal_cache"    
                    else:
                        raise "缓存区数据和环境对不上"    
                else:
                    outputs["place_id"] = place_id
                    undate_task_status(13) 
                    return "tote_0_pal"                 
            elif tote_type==1:
                self.logger.info(f"当前料箱为大欧")  
                #只有两个托盘能放中欧，与缓存区一一对应
                if place_id == "0" or place_id == "1":
                    undate_task_status(13)  
                    return "merge"
                else:
                    raise "无效的大欧放置工作空间"  
            else:
                raise "无效的料箱类型"
