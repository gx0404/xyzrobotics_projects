# -*- coding:utf-8 -*-
'''
Copyright (c) XYZ Robotics Inc. - All Rights Reserved
Unauthorized copying of this file, via any medium is strictly prohibited
Proprietary and confidential
Author: Kun Chen<kun.chen@xyzrobotics.ai>, March 20, 2021
'''
import os
import sys
import time
import threading

import requests

from apps import settings
from apps.log import outside_log
from .inotify_simple import INotify, flags



class NodeLogReader(object):
    """
    """
    READ_CONDITION = flags.MODIFY | flags.MOVED_FROM
    def __init__(self, path):
        super(NodeLogReader, self).__init__()
        self.fr = open(path, 'rb')
        self.filepath = path
        self.filedir, self.filename = self.__check_path(path)
        self.inotify = self.__create_inotify_events()
        
        self.fr.seek(0, 2)
        self.offset = self.fr.tell()


    def __del__(self):
        self.close()

    def __check_path(self, log_path):
        return os.path.split(log_path)

    def __create_inotify_events(self):
        inotify = INotify()
        inotify.add_watch(self.filedir, mask=self.READ_CONDITION)
        return inotify

    def __read_lines(self):
        """ Read length bytes from the file named by filename starting at
        offset 
        """
        # offset = self.offset
        data = ""
        try:
            f = self.fr
            f = open(self.filepath, 'r')      
            f.seek(self.offset)
            data = f.readlines()
            self.offset = f.tell()
        except (OSError, IOError):
            self.offset = 0
            data = []
            print('读取失败')

        return data, self.offset

    def close(self):
        if hasattr(self, 'fr'):
            print("close a file named {0}".format(self.filename))
            self.fr.close()
            self.fr = None

    def read(self, log_path=None, timeout=10000, read_delay=1000):
        if log_path is None:
            log_path = self.filepath
        data = ""
        for event in self.inotify.read(timeout=timeout, read_delay=read_delay):
            if event is None:
                continue
            (wd, mask, cookie, name) = event
            if mask == flags.MODIFY:
                data, self.offset = self.__read_lines()
            elif mask == flags.MOVED_FROM:
                self.offset = 0
                data = "\n"
            break
        
        return data

def node_log_reader_thread(node_log_reader, node_name, requesting, stop_signal):
    """
    Args:
        node_log_reader (NodeLogReader)
        node_name (string)
        stop_signal (Bool): Signal to trigger thread to stop
    """
    from apps import create_app
    app = create_app()
    outside_log.info("node_log_reader [{}] thread started".format(node_name))
    while not stop_signal() :
        data = node_log_reader.read()
        if data:
            app.mp.node_log(node_name, data)
    outside_log.info("node_log_reader [{}] thread stopped".format(node_name))
 

if __name__ == '__main__':
    log_path = sys.argv[-1]
    name = os.path.split(log_path)[-1].split(".")[0]
    print("node name: {}".format(name))
    print("log file: {}".format(log_path))
    print('~'*20)
    node_log_reader = NodeLogReader(log_path)
    
    while True:
        data = node_log_reader.read()
        if data != "":
            print(data)