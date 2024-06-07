
def execute(self, inputs, outputs, gvm):
    self.logger.info("Hello {}".format(self.name))
    cache_pallet_tote_data = inputs["cache_pallet_tote_data"]
    if cache_pallet_tote_data:
        return "cache_pallet"
    else:    
        return "success"
