import sys
from PyQt5.QtWidgets import QApplication, QWidget, QPushButton, QVBoxLayout
import socket


class MyWindow(QWidget):
    def __init__(self):
        super().__init__()
        self.initUI()
        
    def initUI(self):
        self.setWindowTitle('Simple PyQt Example')
        self.setGeometry(100, 100, 300, 200)

        # 创建按钮
        self.button = QPushButton('connect tcp', self)
        self.button.clicked.connect(self.connect_tcp)
        
        # 创建按钮
        self.button = QPushButton('close tcp', self)
        self.button.clicked.connect(self.close_tcp) 
        
        # # 创建按钮
        # self.button = QPushButton('send message', self)
        # self.button.clicked.connect(self.send_message)  # 连接按钮点击事件与处理函数

        
        # 创建布局，并将按钮添加到布局中
        layout = QVBoxLayout()
        layout.addWidget(self.button)

        # 将布局设置为窗口的主布局
        self.setLayout(layout)

    def send_message(self):

        # 发送消息给服务器
        message = 'Hello, server!'
        self.client_socket.sendall(message.encode())

        # # 关闭连接
        # self.client_socket.close()
        print('send message:', message)

    def connect_tcp(self):
        # 服务器配置
        HOST = '127.0.0.1'  # 服务器IP地址
        PORT = 12345        # 服务器端口号   

        # 创建客户端套接字
        self.client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

        # 连接到服务器
        self.client_socket.connect((HOST, PORT))
    
    def close_tcp(self):
        # 关闭连接
        self.client_socket.close()

                


if __name__ == '__main__':
    # 创建应用程序对象
    app = QApplication(sys.argv)
    # 创建窗口对象
    window = MyWindow()
    # 显示窗口
    window.show()
    # 应用程序事件循环
    sys.exit(app.exec_())
