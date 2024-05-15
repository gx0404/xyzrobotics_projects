from flask_socketio import Namespace, flask
from apps.log import outside_log as log


class Connection(Namespace):
    def __init__(self, namespace, socketio):
        super(Connection, self).__init__(namespace)
        self.socketio = socketio

    def on_connect(self):
        log.info("[WebSocket] - {addr} - connected".format(addr=flask.request.remote_addr))

    def on_disconnect(self):
        log.info("[WebSocket] - {addr} - disconnected".format(addr=flask.request.remote_addr))
