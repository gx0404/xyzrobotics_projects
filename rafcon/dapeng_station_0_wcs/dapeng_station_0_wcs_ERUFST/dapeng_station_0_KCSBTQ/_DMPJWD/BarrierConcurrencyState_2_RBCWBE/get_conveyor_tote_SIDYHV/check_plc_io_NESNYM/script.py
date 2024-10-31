
def execute(self, inputs, outputs, gvm):
    self.logger.info("Hello {}".format(self.name))
    gvm.set_variable("motion_payload", None, per_reference=True)
    tote_type = inputs["tote_type"]
    place_id = inputs["place_id"]
    if tote_type==1:
        sku_info = {'height': 0.23, 'length': 0.6, 'weight': 5.0, 'width': 0.4}     
        if place_id not in ["0","1"]:
            raise "大欧无效的放置位"           
    elif tote_type==0:
        sku_info = {'height': 0.23, 'length': 0.4, 'weight': 5.0, 'width': 0.3}    
        if place_id  not in ["4","5"]:
            raise "中欧无效的放置位"          
    else:
        raise "无效的tote_type"
          
    outputs["tote_type"] = tote_type
    outputs["place_id"] = place_id     
    outputs["sku_info"] = sku_info    
    return "success"
