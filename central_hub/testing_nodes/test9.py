#!/usr/bin/python
# -*- coding:utf-8 -*-
'''
Copyright (c) XYZ Robotics Inc. - All Rights Reserved
Unauthorized copying of this file, via any medium is strictly prohibited
Proprietary and confidential
Author: Kun Chen<kun.chen@xyzrobotics.ai>, March 20, 2021
'''

import time
import ctypes
import threading

from central_hub_gui.app.utils import clogger

class WCSThread(threading.Thread):
    """docstring for WCSThread"""
    THREADS = []
    def __init__(self, target, args):
        super(WCSThread, self).__init__(target=target, args=args)

    def kill_thread(self):
        ctypes.pythonapi.PyThreadState_SetAsyncExc(
            ctypes.c_long(self.ident), ctypes.py_object(SystemExit))
        print("线程 {} 被销毁".format(self.getName()))


def create_thread(thread_name, sub_thread_name):
    thread = WCSThread(target=create_sub_thread, args=(thread_name, sub_thread_name, ))
    thread.setName(thread_name)
    thread.daemon = True  # set daemo thread
    WCSThread.THREADS.append(thread)
    thread.start()
    

def create_sub_thread(thread_name, sub_thread_name):
    thread = WCSThread(target=print_content, args=(sub_thread_name, ))
    thread.setName(sub_thread_name)
    # thread.daemon = True  # set daemo thread
    WCSThread.THREADS.append(thread)
    thread.start()
    count = 0
    while 1:
        print("{0}: {1}".format(thread_name, count))
        count += 1
        time.sleep(0.5)



def print_content(thread_name):
    count = 0
    while 1:
        print("{0}: {1}".format(thread_name, count))
        count += 1
        time.sleep(0.5)


def main():
    th_name1 = "AppLogReader"
    th_name2 = "NodeLogReader"
    create_thread(th_name1, th_name2)

    print(">>> I'm sleeping")
    time.sleep(5)
    print(">>> finished")
    # exit(-1)

if __name__ == '__main__':
    main()
