
def execute(self, inputs, outputs, gvm):
    self.logger.info("Hello {}".format(self.name))
    pallet_tote_data_2 = inputs["pallet_tote_data_2"]
    if pallet_tote_data_2:
        return "cache_pallet"
    else:    
        return "success"
