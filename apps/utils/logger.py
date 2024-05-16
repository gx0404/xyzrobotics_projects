#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
    apps.utils.logger
    ~~~~~~~~~~~~~~~~~~~~~

    提供自定义的日志处理器

    Copyright (c) XYZ Robotics Inc. - All Rights Reserved
    Unauthorized copying of this file, via any medium is strictly prohibited
    Proprietary and confidential
    Developer: Chen Kun <kun.chen@xyzrobotics.ai>, June, 2022
"""
import os
import time
import logging
from logging.handlers import RotatingFileHandler


class HMIRotatingFileHandler(RotatingFileHandler):
    """继承原有的logging.handlers.RotatingFileHandler类，实现自定义时间轮换处理器，不
    使用TimedRotatingFileHandler的原因是日志文件名无法直接以日期命名。另外，为了和logging
    模块中的名称方式统一，代码中使用的小驼峰命名法，而没有使用下划线命名。

    """
    __timeFormat__ = '%Y%m%d'   # 设置日志文件更新周期
    __fileFormat__ = '~%Y%m%d.log'  # 文件格式

    def __init__(self, directory='logs', mode='a', maxBytes=0, backupCount=0, encoding=None, delay=False):
        self.directory = directory
        # 当前日志文件时间
        self.atTime = time.strftime(self.__timeFormat__, time.localtime())
        self.filename = None
        self.updateFilename()
        self.createFile()
        RotatingFileHandler.__init__(
            self, self.filename, mode, maxBytes, backupCount, encoding, delay)
        self.redirectStream()

    def updateFilename(self):
        """更新文件名称，根据当前的日期时间和日志名格式对日志文件进行命名
        """
        filename = time.strftime(self.__fileFormat__, time.localtime())
        self.filename = os.path.join(self.directory, filename)

    def createFile(self):
        """检查文件是否存在，如果文件不存在，则创建一个新的文件，如果文件存在，则使用已存在的文件夹。
        """
        if not os.path.exists(self.directory):
            # 使用mkdir只能创建一层目录，多级目录就会失败
            # os.mkdir(self.directory)
            os.makedirs(self.directory)

    def redirectStream(self):
        """关闭原有的stream, 重定义文件名，在emit中会根据baseFilename重新打开一个新的stream
        """
        self.close()    # 关闭stream，并设置stream为None，线程安全
        self.baseFilename = os.path.abspath(self.filename)

    def emit(self, record):
        """Emit a record.如果检测到日志文件的日期和当前日期不相等，则创建一个新的日志，使用的判定
        时间为time.localtime()，没有使用utc时间
        """
        try:
            currentTime = time.strftime(self.__timeFormat__, time.localtime())
            if currentTime != self.atTime:
                self.updateFilename()
                self.createFile()
                self.redirectStream()
                self.atTime = currentTime
            RotatingFileHandler.emit(self, record)
        except Exception:
            self.handleError(record)


def main():
    base_dir = os.path.dirname(os.path.dirname(os.path.dirname(
        os.path.abspath(__file__))))
    print('base_dir:', base_dir)
    logger = logging.getLogger(__name__)
    logger.setLevel(logging.INFO)
    handler = HMIRotatingFileHandler(base_dir)
    logger.addHandler(handler)
    # logger.setFormatter()
    logger.info("123")


if __name__ == '__main__':
    main()
