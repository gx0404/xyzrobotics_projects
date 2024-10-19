import requests
import json,time

def execute(self, inputs, outputs, gvm):
    self.logger.info("Hello {}".format(self.name))
    url = "http://127.0.0.1:7002/api/rafcon/report_pallet_pal_full"
    data = {
    "place_id": inputs["place_id"],
    }
    try:
        response = requests.post(url, json = data).json()
    except requests.exceptions.ConnectionError as e:
        raise Exception("Connection Error: {}".format(e))
    except: 
        raise Exception("Unknown errors occured when requesting response from hmi-back.")
    self.logger.info("response: {}".format(response))
    return "success"
