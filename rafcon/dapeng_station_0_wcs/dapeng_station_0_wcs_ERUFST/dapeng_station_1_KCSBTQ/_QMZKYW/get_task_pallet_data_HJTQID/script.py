import requests
from itertools import combinations

def execute(self, inputs, outputs, gvm):
    url = "http://127.0.0.1:7002/api/rafcon/get_pallet_data"
    try: 
        response = requests.post(url).json()
    except requests.exceptions.ConnectionError as e:
        raise Exception("Connection Error: {}".format(e))
    except: 
        raise Exception("Unknown errors occured when requesting response from hmi-back.")   
    
    if response["error"]!=0:
        raise Exception("Error occured when requesting response from hmi-back.")
    
    current_direction = response["current_direction"]
    outputs["current_direction"] = current_direction
    self.logger.info(f"current_direction is {current_direction}")

    pallet_tote_data = response["pallet_tote_data"]
    outputs["pallet_tote_data"] = pallet_tote_data
    self.logger.info(f"pallet_tote_data is {pallet_tote_data}")

    pick_tote_data = response["pick_tote_data"]
    outputs["pick_tote_data"] = pick_tote_data
    self.logger.info(f"pick_tote_data is {pick_tote_data}")

    cache_pallet_tote_data = response["cache_pallet_tote_data"]   
    outputs["cache_pallet_tote_data"] = cache_pallet_tote_data
    self.logger.info(f"cache_pallet_tote_data is {cache_pallet_tote_data}")

    path = response["path"]   
    outputs["path"] = path
    self.logger.info(f"path is {path}")

    pallet_tote_data_2 = response["pallet_tote_data_2"]   
    outputs["pallet_tote_data_2"] = pallet_tote_data_2
    self.logger.info(f"pallet_tote_data_2 is {pallet_tote_data_2}")
    
    pallet_tote_data_3 = response["pallet_tote_data_3"]   
    outputs["pallet_tote_data_3"] = pallet_tote_data_3
    self.logger.info(f"pallet_tote_data_3 is {pallet_tote_data_3}")

    pallet_tote_data_7 = response["pallet_tote_data_7"]   
    outputs["pallet_tote_data_7"] = pallet_tote_data_7
    self.logger.info(f"pallet_tote_data_7 is {pallet_tote_data_7}")
    
    pallet_tote_data_8 = response["pallet_tote_data_8"]   
    outputs["pallet_tote_data_8"] = pallet_tote_data_8
    self.logger.info(f"pallet_tote_data_8 is {pallet_tote_data_8}")
            
    gvm.set_variable("pallet_tote_data", pallet_tote_data, per_reference=False)
    gvm.set_variable("cache_pallet_tote_data", cache_pallet_tote_data, per_reference=False)    
    return "success"

