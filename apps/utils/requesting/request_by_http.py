import requests
from apps import settings
from apps.log import outside_log as log
from apps.helpers import json_dumper

index = "http://{}".format(settings.HTTP_ADDR)


class Requesting_Http(object):
    def send_log_message(self, data):
        route = "/api/notify/system_log"
        return self.call_backend(data, route)

    def send_node_error(self, data):
        route = "/api/notify/node_error"
        return self.call_backend(data, route)

    def call_backend(self, data, route):
        try:
            url = index + route
            res = requests.post(url=url, json=data)
            res.raise_for_status()
            log.info(u"[WS BY HTTP] - {} - {}".format(route, json_dumper(data)))
            return res.json()
        except Exception:
            log.error(u"[WS BY HTTP] failed to send log message to backend: %s" % url, exc_info=True)
            return {}
