from xyz_env_manager.client import get_planning_environment
from xyz_motion import PlanningEnvironmentRos
def execute(self, inputs, outputs, gvm):
    self.logger.info("Hello {}".format(self.name))
    pl = get_planning_environment()
    planning_env = PlanningEnvironmentRos.from_ros_msg(pl)
    place_id = inputs["place_id"]
    tote_type = inputs["tote_type"]
    unfinished_planned_items = planning_env.get_unfinished_planned_items(place_id)
    self.logger.info(f"放置空间{place_id}放置规划还剩{len(unfinished_planned_items)}个")
    container_items = planning_env.get_container_items(place_id)
    if not unfinished_planned_items:
        if not container_items:
            self.logger.info(f"放置空间无规划")
            return "place_plan_empty"
        else:
            self.logger.info(f"放置空间无规划,但存在料箱,可能是放满了")    
            return "full"
    else:
        if tote_type==0:
            self.logger.info(f"当前料箱为中欧")
            #只有两个托盘能放中欧，与缓存区一一对应
            if place_id == "1":
                container_items = planning_env.get_container_items("7")    
                pallet_tote_data_7 = inputs["pallet_tote_data_7"]
                if pallet_tote_data_7 and container_items:                  
                    if pallet_tote_data_7["to_ws"]!=place_id:
                        raise "数据库缓存区7厂家放置空间号不一致"
                    self.logger.info(f"放置空间{place_id},缓存区有中欧,两个中欧依次放置")
                    outputs["cache_id"] = "7"
                    outputs["place_id"] = place_id
                    return "depal_cache"
                elif (not pallet_tote_data_7) and (not container_items):
                    self.logger.info(f"放置空间{place_id},缓存区无中欧")   
                    outputs["cache_id"] = "7"                 
                    return "pal_cache"    
                else:
                    raise "缓存区数据和环境对不上"        
            elif place_id == "2":
                container_items = planning_env.get_container_items("8")    
                pallet_tote_data_8 = inputs["pallet_tote_data_8"]
                if pallet_tote_data_8 and container_items:                  
                    if pallet_tote_data_8["to_ws"]!=place_id:
                        raise "数据库缓存区8厂家放置空间号不一致"
                    self.logger.info(f"放置空间{place_id},缓存区有中欧,两个中欧依次放置")
                    outputs["cache_id"] = "8"
                    outputs["place_id"] = place_id
                    return "depal_cache"
                elif (not pallet_tote_data_8) and (not container_items):
                    self.logger.info(f"放置空间{place_id},缓存区无中欧")   
                    outputs["cache_id"] = "8"                 
                    return "pal_cache"    
                else:
                    raise "缓存区数据和环境对不上"    
            else:
                raise "无效的中欧放置工作空间"                  
        elif tote_type==1:
            self.logger.info(f"当前料箱为大欧")    
            return "merge"
        else:
            raise "无效的料箱类型"
