import socket

# 服务器配置
HOST = '127.0.0.1'  # 服务器IP地址
PORT = 12345        # 服务器端口号

# 创建服务器套接字
server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

# 绑定服务器地址和端口
server_socket.bind((HOST, PORT))

# 开始监听连接
server_socket.listen()

print('Server listening on {}:{}'.format(HOST, PORT))

# 接受客户端连接
client_socket, client_address = server_socket.accept()

print('Connected to', client_address)

# 接收客户端消息并打印
while True:
    data = client_socket.recv(1024)
    if not data:
        # 接受客户端连接
        client_socket, client_address = server_socket.accept()
        print('Connected to', client_address)
        continue
    print('Received:', data.decode())

# 关闭连接
client_socket.close()
server_socket.close()
