# python3.5
# python-socketio==4.4.0
# python-engineio==3.12.1

import socketio


def start_sio_client():
    index = "http://127.0.0.1:7002"

    # If reconnection is False, inner thread will terminate when the backend is closed
    socket_io = socketio.Client(reconnection=False)

    @socket_io.event
    def system_log(data):
        print("system_log")
        print(data)

    @socket_io.event
    def system_initialization(data):
        print("system_initialization")
        print(data)

    @socket_io.event
    def vision_node(data):
        print("vision_node")
        print(data)

    @socket_io.event
    def node_error(data):
        print("node_error")
        print(data)

    # connect will open a non-deamon thread to listen events
    socket_io.connect(index)
    # when use eventlet， add transports=["websocket"]


print("socketio is listening")
start_sio_client()
