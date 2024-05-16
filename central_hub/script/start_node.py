#!/usr/bin/python
'''
Copyright (c) XYZ Robotics Inc. - All Rights Reserved
Unauthorized copying of this file, via any medium is strictly prohibited
Proprietary and confidential
Author: Kun Chen<kun.chen@xyzrobotics.ai>, January 29, 2021
'''
from __future__ import print_function

import xmlrpc.client
from pprint import pprint

class CentralHubClient(object):
    """docstring for CentralHubClient"""
    def __init__(self):
        super(CentralHubClient, self).__init__()
        self.__server = self.__create_server()
        
    def __create_server(self):
        # url = 'http://{}:{}@localhost:9000'.format(username, password)
        url = 'http://127.0.0.1:9004'
        server = xmlrpc.client.ServerProxy(url)
        return server


    def start_node(self, node_id, restart=False):
        if not node_id:
            node_id = ""
            # node_id = None

        # res = self.__server.start_node(node_id, restart)
        try:
            res = self.__server.start_node(node_id, restart)
            # res = self.__server.start_node_by_dependency(node_id, restart)
        except xmlrpc.client.Fault as e:
            print(e)
            res = {}
        pprint(res, indent=2)

    def restart_node(self, node_id, restart=False):
        if not node_id:
            node_id = ""
        node_id = str(node_id)
        try:
            # print(self.__server.system.listMethods())
            res = self.__server.start_node(node_id, restart)
            # res2 = self.__server.get_dependency()
        except xmlrpc.client.Fault as e:
            print(e)
            res = {}
        finally:
            pprint(res, indent=2)
            # print("-"*50)
            # pprint(res2, indent=2)

    def get_node_stdout(self, node_id):
        byte_data = self.__server.get_node_stdout(node_id)
        # str_data  = str(byte_data, encoding='utf-8')
        if byte_data is not None:
            str_data = str(byte_data)
            print(str_data, end="")

def main():
    ch = CentralHubClient()
    while True:
        text = raw_input("\n---\nstart node: ")
        if text == "q!":
            break
        else:
            text = text.split(" ")
            node_id = text[0]
            if len(text) > 1:
                if text[1]:
                    restart = False
                else:
                    restart = True
            else:
                restart = True
            ch.start_node(node_id, restart)
            # ch.restart_node(node_id, restart)
            # ch.stop_process(node_id)

def test_log():
    ch = CentralHubClient()
    text = raw_input("input node id: ")
    import time
    while True:
        ch.get_node_stdout(text)
        # time.sleep(0.3)

if __name__ == '__main__':
    main()
