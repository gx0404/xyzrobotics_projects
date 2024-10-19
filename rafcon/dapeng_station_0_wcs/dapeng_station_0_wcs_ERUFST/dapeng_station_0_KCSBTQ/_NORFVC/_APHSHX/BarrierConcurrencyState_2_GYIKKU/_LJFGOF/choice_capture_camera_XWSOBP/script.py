from xyz_env_manager.client import get_planning_environment
from xyz_motion import PlanningEnvironmentRos,pose_to_list,SE3
import yaml,json
from xyz_vision_lib.xyz_vision_bridge import XYZVisionBridge

def change_recipe(self,camera_quadrant_list,sku_type,vision_bridge):
    quadrant_camera_id_dict = {0: "L340XP04306C005", 
                                1: "L340XP04404R025",
                                2: "L340XP043090015",
                                3: "L340XP043070014"}
    recipe_path_dict = {
        1:"/home/xyz/xyz_app/app/料箱max视觉/visions/one_camera.yml",
        2:"/home/xyz/xyz_app/app/料箱max视觉/visions/two_camera.yml",
        3:"/home/xyz/xyz_app/app/料箱max视觉/visions/three_camera.yml",
    }
    sku_type_dict = {
        0:["400_300_230"],
        1:["600_400_230"],
    }
    
    recipe_name_dict = {
        1: "one_camera",
        2: "two_camera",
        3: "three_camera",
    }    
    
    camera_calibration_path_parent = "calibration/camera/"
    
    camera_config_path_parent = "/home/xyz/xyz_app/app/料箱max视觉/camera_configs/"
    ##测试用
    #camera_config_path_parent = "/home/xyz/Downloads/dapeng_station_0_20240829095843/L340XP043110024/1724925188909_L340XP043110024_0_camera.json" 
    camera_num = len(camera_quadrant_list)
    recipe_path = recipe_path_dict[camera_num]
    
    #判断是否需要新增配方
    ftr = vision_bridge.async_run(0,"get_recipes_list")
    recipes_list_res = ftr.get()
    recipes_list = recipes_list_res.info
    recipe_name = recipe_name_dict[camera_num]+"_"+f"{camera_quadrant_list}"+"_"+sku_type_dict[sku_type][0]

    self.logger.info(f"当前vision存在配方列表为{recipes_list}")
    self.logger.info(f"配方名称为{recipe_name}")
    if recipe_name not in recipes_list:
        self.logger.info(f"配方{recipe_name}不存在，需要新增")  
        camera_id_list = [quadrant_camera_id_dict[i] for i in camera_quadrant_list]
        with open(recipe_path, "r") as f:
            recipe = yaml.safe_load(f)
            #获取配方所有模块
            model_list = list(recipe.values())[0]
            
            #一个相机配方的更改
            if camera_num==1:
                #遍历所有模块
                for model in model_list:
                    #相机模块
                    if model["model"]["name"] == "XYZCamera":
                        if model["id"]!='{5167e216-0bee-45c0-bcde-4330f578a7b3}':
                            raise Exception("相机模块id错误")
                        model["model"]["params"]["id"] = camera_id_list[0]   
                        model["model"]["params"]["cfg"] = camera_config_path_parent+camera_id_list[0]+".json" 
                        # #测试用
                        # model["model"]["params"]["cfg"] = camera_config_path_parent
                        self.logger.info(f"XYZCamera模块,相机ID更改为{camera_id_list[0] }")
                        self.logger.info(f"XYZCamera模块,相机配置文件更改为{model['model']['params']['cfg']}")
                    #加载相机标定模块    
                    if model["model"]["name"] == "LoadCameraCalibInfo":    
                        if model["id"]!='{78bd81ce-19ef-435b-9a99-869dc3fd09d0}':
                            raise Exception("加载相机标定模块id错误")
                        model["model"]["params"]["calib_file"] = camera_calibration_path_parent+camera_id_list[0]+"/"+"camera_extrinsic.yml"
                        self.logger.info(f"加载相机标定模块,相机标定文件更改为{model['model']['params']['calib_file']}")  
                    #SBM模型模块    
                    if model["model"]["name"] == "SBMEstimator":      
                        if model["id"]!='{5018f7e8-b2eb-469a-aab3-81cba7277373}':
                            raise Exception("SBM模型模块id错误")  
                        model["model"]["params"]["detection_targets"] = sku_type_dict[sku_type] 
                        self.logger.info(f"SBM模型模块,模型更改为{sku_type_dict[sku_type]}")     
                    #计算最高层高度模块    
                    if model["model"]["name"] == "CalculateTopLayerHeight":      
                        if model["id"]!='{87e22c96-a7f9-4daa-92ac-6639e791a3ac}':
                            raise Exception("计算最高层高度模块id错误")    
                        if sku_type == 0:
                            layer_min_points = 2000
                        else:
                            layer_min_points = 4000        
                        model["model"]["params"]["layer_min_points"] = layer_min_points  
                        self.logger.info(f"计算最高层高度模块,顶层最少点云更改为{layer_min_points}")                             
                        
            elif camera_num==2:
                #遍历所有模块
                for model in model_list:
                    #相机模块
                    if model["model"]["name"] == "XYZCamera":
                        if model["id"]=="{4d3282e1-76eb-4c78-87ee-88356b2ca998}":
                            model["model"]["params"]["id"] = camera_id_list[0]   
                            model["model"]["params"]["cfg"] = camera_config_path_parent+camera_id_list[0]+".json" 
                            # #测试用
                            # model["model"]["params"]["cfg"] = camera_config_path_parent
                            self.logger.info(f"XYZCamera模块,相机ID更改为{camera_id_list[0] }")
                            self.logger.info(f"XYZCamera模块,相机配置文件更改为{model['model']['params']['cfg']}")
                        elif model["id"]=="{431c1aae-c6ff-4fc4-b5a2-7cc261eaa2c4}":
                            model["model"]["params"]["id"] = camera_id_list[1]   
                            model["model"]["params"]["cfg"] = camera_config_path_parent+camera_id_list[1]+".json" 
                            # #测试用
                            # model["model"]["params"]["cfg"] = camera_config_path_parent
                            self.logger.info(f"XYZCamera模块,相机ID更改为{camera_id_list[1] }")
                            self.logger.info(f"XYZCamera模块,相机配置文件更改为{model['model']['params']['cfg']}")                            
                        else:
                            raise Exception("相机模块id错误")
                    #加载相机标定模块    
                    if model["model"]["name"] == "LoadCameraCalibInfo":    
                        if model["id"]=="{4dbd820a-a38c-4015-8144-a17955f74613}":   
                            model["model"]["params"]["calib_file"] = camera_calibration_path_parent+camera_id_list[0]+"/"+"camera_extrinsic.yml"
                            self.logger.info(f"加载相机标定模块,相机标定文件更改为{model['model']['params']['calib_file']}")  
                        elif model["id"]=="{5d8614cc-419c-4ecc-8406-0ecff71ff071}":   
                            model["model"]["params"]["calib_file"] = camera_calibration_path_parent+camera_id_list[1]+"/"+"camera_extrinsic.yml"
                            self.logger.info(f"加载相机标定模块,相机标定文件更改为{model['model']['params']['calib_file']}")                         
                        else:
                            raise Exception("加载相机标定模块id错误")    
                    #SBM模型模块    
                    if model["model"]["name"] == "SBMEstimator":      
                        if model["id"]!="{1b5d45ca-94c5-4513-901e-f28396c3c014}":
                            raise Exception("SBM模型模块id错误")
                        model["model"]["params"]["detection_targets"] = sku_type_dict[sku_type] 
                        self.logger.info(f"SBM模型模块,模型更改为{sku_type_dict[sku_type]}")     
                    #计算最高层高度模块    
                    if model["model"]["name"] == "CalculateTopLayerHeight":      
                        if model["id"]!="{50df878c-7926-4142-b82b-6283bc19748a}":
                            raise Exception("计算最高层高度模块id错误") 
                        if sku_type == 0:
                            layer_min_points = 2000
                        else:
                            layer_min_points = 4000        
                        model["model"]["params"]["layer_min_points"] = layer_min_points  
                        self.logger.info(f"计算最高层高度模块,顶层最少点云更改为{layer_min_points}")     
                                                      
            elif camera_num==3:
                #遍历所有模块
                for model in model_list:
                    #相机模块
                    if model["model"]["name"] == "XYZCamera":
                        if model["id"]=="{bb22c145-40c3-477b-baa4-505cc179f8e3}":
                            model["model"]["params"]["id"] = camera_id_list[0]   
                            model["model"]["params"]["cfg"] = camera_config_path_parent+camera_id_list[0]+".json" 
                            # #测试用
                            # model["model"]["params"]["cfg"] = camera_config_path_parent
                            self.logger.info(f"XYZCamera模块,相机ID更改为{camera_id_list[0] }")
                            self.logger.info(f"XYZCamera模块,相机配置文件更改为{model['model']['params']['cfg']}")
                        elif model["id"]=="{626dee08-21f7-4752-abe8-858e842c9403}":
                            model["model"]["params"]["id"] = camera_id_list[1]   
                            model["model"]["params"]["cfg"] = camera_config_path_parent+camera_id_list[1]+".json" 
                            # #测试用
                            # model["model"]["params"]["cfg"] = camera_config_path_parent
                            self.logger.info(f"XYZCamera模块,相机ID更改为{camera_id_list[1] }")
                            self.logger.info(f"XYZCamera模块,相机配置文件更改为{model['model']['params']['cfg']}")     
                        elif model["id"]=="{04da14bd-afef-46fa-bf6c-9b61ac715857}":
                            model["model"]["params"]["id"] = camera_id_list[2]   
                            model["model"]["params"]["cfg"] = camera_config_path_parent+camera_id_list[2]+".json" 
                            # #测试用
                            # model["model"]["params"]["cfg"] = camera_config_path_parent
                            self.logger.info(f"XYZCamera模块,相机ID更改为{camera_id_list[2]}")
                            self.logger.info(f"XYZCamera模块,相机配置文件更改为{model['model']['params']['cfg']}")                                                 
                        else:
                            raise Exception("相机模块id错误") 
                    #加载相机标定模块    
                    if model["model"]["name"] == "LoadCameraCalibInfo":    
                        if model["id"]=="{562400dd-f1d1-417e-8852-b3a549371932}":   
                            model["model"]["params"]["calib_file"] = camera_calibration_path_parent+camera_id_list[0]+"/"+"camera_extrinsic.yml"
                            self.logger.info(f"加载相机标定模块,相机标定文件更改为{model['model']['params']['calib_file']}")  
                        elif model["id"]=="{ac3ab389-7317-4c2d-9053-0b7595477352}":   
                            model["model"]["params"]["calib_file"] = camera_calibration_path_parent+camera_id_list[1]+"/"+"camera_extrinsic.yml"
                            self.logger.info(f"加载相机标定模块,相机标定文件更改为{model['model']['params']['calib_file']}")     
                        elif model["id"]=="{b7ad046b-c208-496b-abf5-2f51741337a7}":   
                            model["model"]["params"]["calib_file"] = camera_calibration_path_parent+camera_id_list[2]+"/"+"camera_extrinsic.yml"
                            self.logger.info(f"加载相机标定模块,相机标定文件更改为{model['model']['params']['calib_file']}")                                              
                        else:
                            raise Exception("加载相机标定模块id错误")     
                    #SBM模型模块    
                    if model["model"]["name"] == "SBMEstimator":      
                        if model["id"]!="{1b8d0c00-49f8-4c64-8092-39b36b98c100}":
                            raise Exception("SBM模型模块id错误") 
                        model["model"]["params"]["detection_targets"] = sku_type_dict[sku_type] 
                        self.logger.info(f"SBM模型模块,模型更改为{sku_type_dict[sku_type]}")     
                    #计算最高层高度模块    
                    if model["model"]["name"] == "CalculateTopLayerHeight":      
                        if model["id"]!="{fcec4402-4351-4ea0-8967-b2a8fc2c5265}":
                            raise Exception("计算最高层高度模块id错误")
                        if sku_type == 0:
                            layer_min_points = 2500
                        else:
                            layer_min_points = 4000        
                        model["model"]["params"]["layer_min_points"] = layer_min_points  
                        self.logger.info(f"计算最高层高度模块,顶层最少点云更改为{layer_min_points}")                                
        with open(recipe_path, 'w',encoding='utf-8') as f:
            load_recipe = {recipe_name:list(recipe.values())[0]}
            yaml.safe_dump(load_recipe, f,allow_unicode=True)                                                                 
            self.logger.info(f"更改配方文件成功,配方文件为{recipe_path}")  
            
        ftr = vision_bridge.async_run(0,"set_recipe_from_file",info=json.dumps({"recipe_name":recipe_name,"file_path":recipe_path}))    
        load_recipe_res = ftr.get()
        self.logger.info(f"倒入更改后的配方文件结果为{load_recipe_res.error_msg}")
        ftr = vision_bridge.async_run(0,"set_recipe",recipe_name)
        change_recipe_res = ftr.get()
        self.logger.info(f"切换配方结果为{change_recipe_res.error_msg}")
        return True
    else:
        self.logger.info(f"配方{recipe_name}已存在,无需倒入配方")
        ftr = vision_bridge.async_run(0,"set_recipe",recipe_name)
        change_recipe_res = ftr.get()
        self.logger.info(f"切换配方结果为{change_recipe_res.error_msg}")        
        return False

def execute(self, inputs, outputs, gvm):
    self.logger.info("Hello {}".format(self.name))
    pick_ws_id = "0"
    cache_ws_id = "1"
    planning_env_msg = get_planning_environment()
    planning_env = PlanningEnvironmentRos.from_ros_msg(planning_env_msg)
    
    if not gvm.get_variable("vision_bridge", per_reference=True, default=None):
        vision_bridge = XYZVisionBridge()
        gvm.set_variable("vision_bridge", vision_bridge, per_reference = True)
    else:
        vision_bridge = gvm.get_variable("vision_bridge", per_reference = True) 
           
    pick_workspace = planning_env.get_workspace_ros(pick_ws_id)
    #获取环境箱子   
    container_items = planning_env.get_container_items(pick_ws_id)     
 
    #wcs订单里有目标点的箱子,通过视觉和wcs订单对箱子赋值，除了缓存区的箱子都是目标箱子
    pick_tote_items = list(filter(lambda x:x.additional_info.values[-1]!=cache_ws_id,container_items))    
    
    #通过抓取计算得到的路径，指定抓取箱子        
    plan_path = gvm.get_variable("plan_path", per_reference=False, default=None)
    #plan_path = []
    if not pick_tote_items and not plan_path:
        self.logger.info("拣配区域没有箱子,且没有抓取路径,拣配抓取完成")
        return "pick_complete"  
    elif pick_tote_items and not plan_path:
        self.logger.info("当前拣配路径为空，需要搜索下一个拣配路径")
        return "find_pick_path"   
    elif plan_path and not pick_tote_items:
        raise Exception(f"奇怪的bug,拣配路径{plan_path}不为空，但拣配区域没有箱子")                 
    elif plan_path and pick_tote_items:
        #通过拣配路径，得到下次抓取的箱子
        next_pick_item_id = plan_path[0]
        self.logger.info(f"下次抓取的箱子ID为{next_pick_item_id}")
        next_pick_item = list(filter(lambda x:x.additional_info.values[-3]==next_pick_item_id,container_items))[0]
        if not next_pick_item:
            raise Exception("未知错误,通过ID无法搜索到下次抓取的箱子")
        tf_base_PickWorkspace = pick_workspace.get_bottom_pose()
        tf_base_item = SE3(pose_to_list(next_pick_item.origin))
        tf_PickWorkspace_item = tf_base_PickWorkspace.inv() * tf_base_item
        tf_PickWorkspace_item_pose = tf_PickWorkspace_item.xyz_quat
        self.logger.info(f"下次抓取的箱子相对于托盘的位姿为{tf_PickWorkspace_item_pose}")
        next_item_x = tf_PickWorkspace_item_pose[0]
        next_item_y = tf_PickWorkspace_item_pose[1]
        
        camera_quadrant_list = []
        if next_item_x>=0 and next_item_y>=0:
            self.logger.info("下次抓取的箱子在拣配区第一象限")
            camera_quadrant_list.append(0)
                        
            # if next_item_x<0.2:
            #     self.logger.info("下次抓取的箱子在拣配区第一象限,靠近第二象限")
            #     camera_quadrant_list.append(1)
            camera_quadrant_list.append(1)
            if next_item_y<0.25:
                self.logger.info("下次抓取的箱子在拣配区第一象限,靠近第四象限")
                camera_quadrant_list.append(3)      
                 
        elif next_item_x<=0 and next_item_y>=0:
            self.logger.info("下次抓取的箱子在拣配区第二象限")
            camera_quadrant_list.append(1)
            # if next_item_x>-0.2:
            #     self.logger.info("下次抓取的箱子在拣配区第二象限,靠近第一象限")
            #     camera_quadrant_list.append(0)
            camera_quadrant_list.append(0)
            if next_item_y<0.25:
                self.logger.info("下次抓取的箱子在拣配区第二象限,靠近第三象限")
                camera_quadrant_list.append(2)  
 
        elif next_item_x<=0 and next_item_y<=0:
            self.logger.info("下次抓取的箱子在拣配区第三象限")
            camera_quadrant_list.append(2)
            # if next_item_x>-0.25:
            #     self.logger.info("下次抓取的箱子在拣配区第三象限,靠近第四象限")
            #     camera_quadrant_list.append(3)
            camera_quadrant_list.append(3)
            if next_item_y>-0.25:
                self.logger.info("下次抓取的箱子在拣配区第三象限,靠近第二象限")
                camera_quadrant_list.append(1)
                
        elif next_item_x>=0 and next_item_y<=0:
            self.logger.info("下次抓取的箱子在拣配区第四象限")
            camera_quadrant_list.append(3)
            # if next_item_x<0.25:
            #     self.logger.info("下次抓取的箱子在拣配区第四象限,靠近第三象限")
            #     camera_quadrant_list.append(2)
            camera_quadrant_list.append(2)
            if next_item_y>-0.25:
                self.logger.info("下次抓取的箱子在拣配区第四象限,靠近第一象限")
                camera_quadrant_list.append(0)  
                
        self.logger.info(f"需要启动的象限相机为: {camera_quadrant_list}")  
        sku_info = inputs["sku_info"]
        sku_dimension = [sku_info["length"],sku_info["width"],sku_info["height"]]
        sku_dimension = list(map(lambda x:round(x,3),sku_dimension))
        
        if sku_dimension==[0.4,0.3,0.23]:
            sku_type = 0
        elif sku_dimension==[0.6,0.4,0.23]:
            sku_type = 1
        else:
            raise Exception("无效的尺寸")   

        # """"测试"""
        # camera_quadrant_list_all = gvm.get_variable("camera_quadrant_list_all", per_reference=False, default=None)
        # if not camera_quadrant_list_all:
        #     from itertools import combinations
        #     init_list = [0,1,2,3]
        #     camera_quadrant_list_all = []
        #     for i in list(combinations(init_list, 1)):
        #         camera_quadrant_list_all.append(i)    
        #     for i in list(combinations(init_list, 2)):
        #         camera_quadrant_list_all.append(i) 
        #     for i in list(combinations(init_list, 3)):
        #         camera_quadrant_list_all.append(i)         
      
        # camera_quadrant_list = camera_quadrant_list_all[0]
        # camera_quadrant_list_all.pop(0)
        # gvm.set_variable("camera_quadrant_list_all", camera_quadrant_list_all, per_reference = False)
        # self.logger.info(f"camera_quadrant_list: {camera_quadrant_list}")    
        # """测试"""  
        
        camera_quadrant_list = sorted(camera_quadrant_list)
        change_recipe(self,camera_quadrant_list,sku_type,vision_bridge)                                                
        outputs["camera_num"] = len(camera_quadrant_list)
        outputs["next_pick_item_id"] = next_pick_item_id
        return "capture_images"

