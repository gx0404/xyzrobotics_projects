#!/usr/bin/env python
# -*- coding: utf-8 -*-
'''
Copyright (c) XYZ Robotics Inc. - All Rights Reserved
Unauthorized copying of this file, via any medium is strictly prohibited
Proprietary and confidential
Author: Xiao Ao <ao.xiao@xyzrobotics.ai>, January, 2021
'''
import os
import re
import datetime


class ClearLog(object):
    """
    1. 作用: 删除文件夹下部分日志文件,使得该文件夹下只有指定天数(可以不连续)内的日志文件

    2. 可处理的文件形式: ①{%4d}{%2d}{%2d}.{%s}, 前面可以有"~", 后面可以有".{%d}"
                      ②日期必须是有效日期,否则忽略
                      ③日期不可是未来日期,否则忽略
                      ④20210317.log与20210317.log.1当做一天处理

    3. 可处理的范围: main_path目录下所有一级子目录下的日志文件

    4. 使用方法: ClearLog(main_path=abs_path, limit=10, file_suffix=("log",)).run()

    Args:
        main_path: str, abs dirpath of all log subpath
        limit: int, the number of logs in each subdir
        file_suffix: tuple, filter logs with the suffix in file_suffix

    """
    def __init__(self, main_path, limit=30, file_suffix=("log", )):

        self._dir_list = []
        self.main_path = main_path
        self.limit = limit
        self.file_suffix = file_suffix
        self._compile_obj = re.compile(
            "^~*(?P<year>\d{4})(?P<month>\d{2})(?P<day>\d{2})\.(?P<tail>.*)")
        self._read_cnf()
        self._today = datetime.date.today()

    def _read_cnf(self):
        if not os.path.exists(self.main_path):
            # os.makedirs(self.main_path)
            # self._dir_list will be empty, which is safe
            return
        for dirname in os.listdir(self.main_path):
            abs_dir = os.path.join(self.main_path, dirname)
            if os.path.isdir(abs_dir):
                self._dir_list.append(abs_dir)

    def _is_target(self, match_obj):
        if not match_obj:
            return False

        tail = match_obj.group("tail")
        tail_suffix = tail.split('.')
        if tail_suffix[0] not in self.file_suffix:
            return False

        # only accept log like 20210202.log or 20210202.log.1
        if len(tail_suffix) != 1:
            if len(tail_suffix) != 2:
                return False
            if not tail_suffix[1].isdigit():
                return False

        # reject logs made up of invalid date
        try:
            year = int(match_obj.group("year"))
            month = int(match_obj.group("month"))
            day = int(match_obj.group("day"))
            date_obj = datetime.date(year=year, month=month, day=day)
        except Exception:
            return False
        if date_obj > self._today:
            return False
        return True

    def _collect_all_log(self, dir_path):
        # dir_path is abspath
        ret = {}
        if not os.path.exists(dir_path):
            print("dir path to clear log dosen't exist\n%s" % dir_path)
            return ret
        for element in os.listdir(dir_path):
            abs_path = os.path.join(dir_path, element)
            # TODO a dir may exist more than one type log, such as ~20201212.txt and 20201212.txt
            if os.path.isfile(abs_path):
                match_obj = self._compile_obj.search(element)
                if self._is_target(match_obj):
                    key = "{year}{month}{day}".format(
                        year=match_obj.group("year"),
                        month=match_obj.group("month"),
                        day=match_obj.group("day"))
                    ret.setdefault(key, []).append(element)
        return ret

    def _delete_file(self, limit, dir_path, date_dict):
        day_class = list(date_dict.keys())
        day_class.sort(reverse=True)  # descent
        delete_days = day_class[limit:]
        for date_str in delete_days:
            for file_path in date_dict[date_str]:
                abs_file_dir = os.path.join(dir_path, file_path)
                try:
                    os.remove(abs_file_dir)
                    print("remove: %s" % abs_file_dir)
                except Exception:
                    print("fail to remove %s" % abs_file_dir)

    def run(self):
        for abs_path in self._dir_list:
            date_dict = self._collect_all_log(abs_path)
            self._delete_file(self.limit, abs_path, date_dict)
