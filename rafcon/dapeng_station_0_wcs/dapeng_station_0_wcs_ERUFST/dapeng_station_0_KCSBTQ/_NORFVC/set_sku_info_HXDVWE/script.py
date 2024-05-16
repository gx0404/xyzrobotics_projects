                
from xyz_env_manager.client import switch_tool
from xyz_motion import RobotDriver
from xyz_env_manager.client import get_planning_environment,modify_primitive_group_of_environment
from xyz_motion import PlanningEnvironmentRos
from xyz_env_manager.client import modify_workspace_of_environment
from xyz_env_manager.client import clear_planned_items
def execute(self, inputs, outputs, gvm):
    """ 
    Output a constant value.

    Args:
        Inputs Data:
            sku_info

        Outputs Data:
            sku_info (dict): Default value (None).
                output sku_info

        Gvm: 
            None
            

    Properties Data:
        comment, (unicode): Default value (u"set sku info").
            The comment of this state. This will show in state's GUI block.
        sku_info:
                length:(float)
                width:(float)
                height:(float)
                weight:(float)
                barcode_direction:(int)
                                    0: no barcode
                                    1: barcode on two long side
                                    2: barcode on one long side
                                    3: barcode on two short side
                                    4: barcode on one short side

        output_value, (any type, default int):  Default value (0).
            This value of output_value will becomes output.

    Outcomes:
        0: success
        -1: aborted
        -2: preempted
    """
    self.logger.info("Running {}({})".format(self.name, self.unique_id))
    self.logger.info(self.smart_data["comment"])
    sku_info_wcs = inputs["sku_info"]
    sku_info_default = self.smart_data["sku_info"]

    barcode_direction = sku_info_default["barcode_direction"]
    if barcode_direction not in range(5):
        raise Exception("barcode direction should be 0, 1, 2, 3, 4 (条码朝向值应该为 0, 1, 2, 3, 4)")
    if sku_info_wcs:
        sku_info_default["length"] = sku_info_wcs["length"]
        sku_info_default["width"] = sku_info_wcs["width"]
        sku_info_default["height"] = sku_info_wcs["height"]
        sku_info_default["weight"] = sku_info_wcs["weight"]
    else:
        self.logger.warning("Please notice that input sku_info is None!")

    sku_update_setting = self.properties_data['sku_update_setting']
    if sku_update_setting['update_sku_lwh_from_vision'] and sku_update_setting['update_sku_maxheight']:
        raise Exception("Can not choose more than one mode to update sku.")
    
    # 需要根据视觉二次拍照的结果更新sku长宽高
    if sku_update_setting['update_sku_lwh_from_vision']:
        sku_lwh_vision = gvm.get_variable("sku_lwh_vision", per_reference=False, default=None)
        if sku_lwh_vision:
            sku_info_default["length"] = sku_lwh_vision[0]
            sku_info_default["width"] = sku_lwh_vision[1]
            sku_info_default["height"] = sku_lwh_vision[2]
            self.logger.info("sku_info的长宽高将根据视觉二次拍照的尺寸进行更新。")
        else:
            self.logger.warning("没有sku_lwh_vision的全局变量，sku_info的长宽高将不根据视觉二次拍照的尺寸结果更新。")

    
    # 需要给出技术协议约定的最大尺寸高度的纸箱
    if sku_update_setting['update_sku_maxheight']:
        if sku_update_setting["sku_maxheight"] < sku_info_default["height"]:
            raise Exception("This maxheight is wrong.")
        sku_info_default["height"] = sku_update_setting["sku_maxheight"]
        self.logger.info("sku_info的高将根据技术协议约定的最大尺寸高度进行更新。")
        
    pl = get_planning_environment()    
    pl_workspace_2 = pl.workspaces[2]
    pl_workspace_2.bottom_pose.z = -1.282
    pl_workspace_2.dimensions = (1.22, 0.83, 1.62)   
    pl_workspace_2.pallet.length = 1.22
    pl_workspace_2.pallet.width = 0.83
    pl_workspace_2.pallet.top_origin.z = -1.282
    
    pl_workspace_3 = pl.workspaces[3] 
    pl_workspace_3.bottom_pose.z = -1.282     
    pl_workspace_3.dimensions = (1.22, 0.83, 1.62)    
    pl_workspace_3.pallet.length = 1.25
    pl_workspace_3.pallet.width = 0.83  
    pl_workspace_3.pallet.top_origin.z = -1.282  
    
    modify_workspace_of_environment(pl_workspace_2)
    modify_workspace_of_environment(pl_workspace_3) 
    
    # pl_workspace_1 = pl.workspaces[1]
    # pl_workspace_1.bottom_pose.z = -1.271
    # pl_workspace_1.pallet.top_origin.z = -1.271    
    # modify_workspace_of_environment(pl_workspace_1)


    sku_dimension = [sku_info_default["length"],sku_info_default["width"],sku_info_default["height"]]
    sku_dimension = list(map(lambda x:round(x,3),sku_dimension))
    rob_driver = RobotDriver(0)
    if sku_dimension==[0.405,0.303,0.16]:
        sku_info_default["overlapping_heihgt"] = 0.01
        gvm.set_variable("depal_scan_code", 1, per_reference=False)
        gvm.set_variable("cache_scan_code", 2, per_reference=False)
        gvm.set_variable("merge_cache_scan_code", 3, per_reference=False)
        sku_info_default["row"] = 9
        switch_tool("0","tool0")
        rob_driver.set_digital_output(1,0)
        rob_driver.set_digital_output(13,1)
    elif sku_dimension==[0.4,0.3,0.23]:
        sku_info_default["overlapping_heihgt"] = 0.012
        gvm.set_variable("depal_scan_code", 1, per_reference=False)
        gvm.set_variable("cache_scan_code", 2, per_reference=False)
        gvm.set_variable("merge_cache_scan_code", 3, per_reference=False)
        sku_info_default["row"] = 9
        switch_tool("0","tool1")
        rob_driver.set_digital_output(1,0)
        rob_driver.set_digital_output(13,1)
        #笼车障碍物清除
        #更新笼车围栏障碍物
        collision_pallet_list = ["collision_pallet_2","collision_pallet_3"]
        for collision_objcet in pl.collision_objects:
            if collision_objcet.name in collision_pallet_list:
                self.logger.info(f"更新笼车围栏障碍物{collision_objcet.name}")
                # collision_objcet.primitives[0].dimensions = [0.01,0.01,0.01] 
                collision_objcet.primitives[0].dimensions = [0.87, 0.07,1.7]
                modify_primitive_group_of_environment(collision_objcet)          
    elif sku_dimension==[0.6,0.4,0.23]:
        sku_info_default["overlapping_heihgt"] = 0.01
        gvm.set_variable("depal_scan_code", 4, per_reference=False)
        gvm.set_variable("cache_scan_code", 5, per_reference=False)
        gvm.set_variable("merge_cache_scan_code", 6, per_reference=False)
        sku_info_default["row"] = 5
        switch_tool("0","tool2")
        rob_driver.set_digital_output(1,1)
        rob_driver.set_digital_output(13,0)
        #笼车障碍物清除
        pl = get_planning_environment()
        #更新笼车围栏障碍物
        collision_pallet_list = ["collision_pallet_2","collision_pallet_3"]
        for collision_objcet in pl.collision_objects:
            if collision_objcet.name in collision_pallet_list:
                self.logger.info(f"更新笼车围栏障碍物{collision_objcet.name}")
                collision_objcet.primitives[0].dimensions = [0.87, 0.07,1.7]
                modify_primitive_group_of_environment(collision_objcet)             
    else:
        raise "无效的尺寸"  
    gvm.set_variable("row", sku_info_default["row"], per_reference=False)
    gvm.set_variable("motion_payload", None, per_reference=True)
    gvm.set_variable("move_camera_flag", True, per_reference=False)
    gvm.set_variable("update_box_ids", [], per_reference=False)  
    clear_planned_items("6")
    outputs["sku_info"] = sku_info_default        
    return "success"