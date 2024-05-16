#!/usr/bin/env python
# -*- coding:utf-8 -*-
'''
Copyright (c) XYZ Robotics Inc. - All Rights Reserved
Unauthorized copying of this file, via any medium is strictly prohibited
Proprietary and confidential
Author: Jiawei Zou<jiawei.zou@xyzrobotics.ai>, January 29, 2021
'''
import os
import re

DEFAULT_SUPERVISORD_CONF_PATH = "/home/xyz/xyz_app/central_hub/supervisord/supervisord.conf"


path = os.popen('echo $PATH').read().replace("\n", "")
ld_library_path = os.popen('echo $LD_LIBRARY_PATH').read().replace("\n", "")
python_path = os.popen('echo $PYTHONPATH').read().replace("\n", "")

new_path = 'environment=PATH="{}"'.format(path)
new_li_library_path = 'LD_LIBRARY_PATH="{}"'.format(ld_library_path)
new_python_path = 'PYTHONPATH="{}"'.format(python_path)

pattern_list = [
    (new_path, r'environment\s{0,3}=\s{0,3}PATH\s{0,3}=\s{0,3}"(.*?)"'),
    (new_li_library_path, r'LD_LIBRARY_PATH\s{0,3}=\s{0,3}"(.*?)"'),
    (new_python_path, r'PYTHONPATH\s{0,3}=\s{0,3}"(.*?)"')
]

with open(DEFAULT_SUPERVISORD_CONF_PATH, "r") as fr:
    read_data = fr.read()
    replace_text = read_data
    for i in range(len(pattern_list)):
        new_data = re.sub(pattern_list[i][1], pattern_list[i][0], replace_text)


with open(DEFAULT_SUPERVISORD_CONF_PATH, "wb") as fw:
    b = fw.write(new_data)