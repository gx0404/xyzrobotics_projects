#  -*- coding: utf-8 -*-
#  -*- coding: utf-8 -*-
#  Copyright (c) XYZ Robotics Inc. - All Rights Reserved
#  Unauthorized copying of this file, via any medium is strictly prohibited
#  Proprietary and confidential
#  Author: Yuhang Wu <yuhang.wu@xyzrobotics.ai>, 2022/10/8 下午3:41
from typing import Optional


def change_locale(locale: Optional[str] = None):
    """改变语言"""
    try:
        from flask_babel import refresh
    except ImportError as e:
        raise e
    from apps import cached_app, settings

    app = cached_app()
    app.config["BABEL_DEFAULT_LOCALE"] = locale or settings.LANGUAGE.name
    refresh()


def gettext(msg: str) -> str:
    """翻译"""
    try:
        from flask_babel import _
    except ImportError as e:
        raise e

    return _(msg)
