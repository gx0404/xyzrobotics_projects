import time
def execute(self, inputs, outputs, gvm):
    self.logger.info("Hello {}".format(self.name))
    path = inputs["path"]
    if not path:
        self.logger.info("path is empty")
        time.sleep(1)
        return "wait"
    else:
        place_id = path["place_id"]
        tote_type = path["tote_type"]
        outputs["place_id"] = place_id
        outputs["tote_type"] = tote_type
        outputs["path"] = {}           
        return "success"
