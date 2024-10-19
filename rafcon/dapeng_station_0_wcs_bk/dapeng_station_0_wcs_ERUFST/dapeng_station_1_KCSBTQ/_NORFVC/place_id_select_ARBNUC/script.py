
def execute(self, inputs, outputs, gvm):
    self.logger.info("Hello {}".format(self.name))
    place_id = inputs["place_id"]
    sku_info = inputs["sku_info"]
    sku_dimension = [sku_info["length"],sku_info["width"],sku_info["height"]]
    sku_dimension = list(map(lambda x:round(x,3),sku_dimension))
    
    if place_id == "6":
        outputs["place_pose_by_scan_code"] = False
        sku_info["barcode_direction"] = 4
        
        if sku_dimension==[0.400,0.300,0.23]:
            outputs["barcode_point_to"] = "-y"
        elif sku_dimension==[0.600,0.400,0.23]:
            outputs["barcode_point_to"] = "+x"        
        else:
            raise "无效的尺寸"        
        outputs["sku_info"] = sku_info
    elif place_id == "1":
        outputs["place_pose_by_scan_code"] = True
        sku_info["barcode_direction"] = 0
        outputs["sku_info"] = sku_info        
    elif place_id == "2" or place_id == "3":
        outputs["place_pose_by_scan_code"] = True
        sku_info["barcode_direction"] = 4
        outputs["sku_info"] = sku_info    
        return "other_pallet"      
    else:
        outputs["place_pose_by_scan_code"] = False
        sku_info["barcode_direction"] = 0
        outputs["sku_info"] = sku_info          
    return "success"        

