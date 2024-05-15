# -*- coding: utf-8 -*-
"""
Copyright (c) XYZ Robotics Inc. - All Rights Reserved
Unauthorized copying of this file, via any medium is strictly prohibited
Proprietary and confidential
Author: Yuhang Wu <yuhang.wu@xyzrobotics.ai>, 2022-04-12
"""
import sys
import typing
import warnings


with warnings.catch_warnings():
    # suppress warning.
    warnings.simplefilter(action='ignore', category=FutureWarning)
    from .settings import init_settings

settings = init_settings()

# 将当前启动的 wcs_adaptor 目录加入到导包列表
sys.path.insert(0, str(settings.GENERAL_CONFIG_DIR.parent))

# TODO(YuhangWu):
#   全局 Python 解释器比之前多了这个包路径, xyz-bin-packing 将会从这个路径下导包
sys.path.insert(0, "/opt/xyz/lib/python/dist-packages")

# 禁用警告信息
# https://stackoverflow.com/questions/65293542/how-to-resolve-gdk-critical-171303-280-gdk-cursor-new-for-display-assert
import matplotlib
matplotlib.use('Agg')

from .app import create_app
from .app import cached_app
from .app import Application
from .core.simple_blueprint import SimpleBlueprint
from .models import db
from .exceptions import XYZBaseError
from .globals import mp
from .globals import hmi_msg_pusher
from .globals import requesting
from .globals import socketio
from .globals import hub_client
from .globals import openapi
from .helpers import make_json_response
from .helpers import json_dumper
from .helpers import record_error
from .log import hmi_log
from .log import wcs_log
from .log import plc_log
from .log import customized_log
from .log import xtf_log
from .log import get_logger
from .services.message_push import HMIMessagePusher
from .utils.i18n import gettext as _
from .utils import dt as datetime
from .utils.requesting.request_by_ws import Requesting
from .utils.validation import validator
from .utils.validation import validate
from .schemas import GetResponseSchema
from .schemas import ListResponseSchema
from .base_app.flask_hook import req_log as catch_log
