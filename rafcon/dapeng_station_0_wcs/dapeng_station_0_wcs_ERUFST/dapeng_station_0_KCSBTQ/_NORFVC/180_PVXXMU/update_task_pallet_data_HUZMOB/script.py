import requests

def execute(self, inputs, outputs, gvm):

    data = {}

    if not inputs["current_direction"] is None:
        data["current_direction"] = inputs["current_direction"]  
    if not inputs["pallet_tote_data"] is None:
        data["pallet_tote_data"] = inputs["pallet_tote_data"] 
    if not inputs["pick_tote_data"] is None:
        data["pick_tote_data"] = inputs["pick_tote_data"] 
    if not inputs["cache_pallet_tote_data"] is None:
        data["cache_pallet_tote_data"] = inputs["cache_pallet_tote_data"]      
    if not inputs["pallet_tote_data_2"] is None:
        data["pallet_tote_data_2"] = inputs["pallet_tote_data_2"]   
    if not inputs["pallet_tote_data_3"] is None:
        data["pallet_tote_data_3"] = inputs["pallet_tote_data_3"]       
    if not inputs["pallet_tote_data_7"] is None:
        data["pallet_tote_data_7"] = inputs["pallet_tote_data_7"]   
    if not inputs["pallet_tote_data_8"] is None:
        data["pallet_tote_data_8"] = inputs["pallet_tote_data_8"]       
    if not inputs["path"] is None:
        data["path"] = inputs["path"]                           
    self.logger.info(f"Update pallet data is {data}")
    
    url = "http://127.0.0.1:7002/api/rafcon/update_pallet_data"
    try: 
        response = requests.post(url,json=data).json()
    except requests.exceptions.ConnectionError as e:
        raise Exception("Connection Error: {}".format(e))
    except: 
        raise Exception("Unknown errors occured when requesting response from hmi-back.")   
    
    if response["error"]!=0:
        raise Exception("Error occured when requesting response from hmi-back.")

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
    path = response["path"]
    self.logger.info(f"path is {path}")
    current_direction = response["current_direction"]
    self.logger.info(f"current_direction is {current_direction}")
    pallet_tote_data = response["pallet_tote_data"]
    self.logger.info(f"pallet_tote_data is {pallet_tote_data}")
    pick_tote_data = response["pick_tote_data"]
    self.logger.info(f"pick_tote_data is {pick_tote_data}")
    cache_pallet_tote_data = response["cache_pallet_tote_data"]   
    self.logger.info(f"cache_pallet_tote_data is {cache_pallet_tote_data}")
    pallet_tote_data_2 = response["pallet_tote_data_2"]   
    self.logger.info(f"pallet_tote_data_2 is {pallet_tote_data_2}")  
    pallet_tote_data_3 = response["pallet_tote_data_3"]   
    self.logger.info(f"pallet_tote_data_3 is {pallet_tote_data_3}")    
    pallet_tote_data_7 = response["pallet_tote_data_7"]   
    self.logger.info(f"pallet_tote_data_7 is {pallet_tote_data_7}")     
    pallet_tote_data_8 = response["pallet_tote_data_8"]   
    self.logger.info(f"pallet_tote_data_8 is {pallet_tote_data_8}")       
    gvm.set_variable("pallet_tote_data", pallet_tote_data, per_reference=False)
    gvm.set_variable("pick_tote_data", pick_tote_data, per_reference=False)
    return "success"

