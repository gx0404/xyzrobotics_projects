                
from xyz_env_manager.client import switch_tool
from xyz_motion import RobotDriver
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
    elif sku_dimension==[0.6,0.4,0.23]:
        sku_info_default["overlapping_heihgt"] = 0.01
        gvm.set_variable("depal_scan_code", 4, per_reference=False)
        gvm.set_variable("cache_scan_code", 5, per_reference=False)
        gvm.set_variable("merge_cache_scan_code", 6, per_reference=False)
        sku_info_default["row"] = 5
        switch_tool("0","tool2")
        rob_driver.set_digital_output(1,1)
        rob_driver.set_digital_output(13,0)
    else:
        raise "无效的尺寸"   
    gvm.set_variable("row", sku_info_default["row"], per_reference=False)
    outputs["sku_info"] = sku_info_default 
    from xyz_env_manager.client import clear_planned_items
    gvm.set_variable("motion_payload", None, per_reference=True)
    clear_planned_items("0")       
    return "success"