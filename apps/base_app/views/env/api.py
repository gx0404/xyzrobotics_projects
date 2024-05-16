#  -*- coding: utf-8 -*-
#  Copyright (c) XYZ Robotics Inc. - All Rights Reserved
#  Unauthorized copying of this file, via any medium is strictly prohibited
#  Proprietary and confidential
#  Author: Yuhang Wu <yuhang.wu@xyzrobotics.ai>, 2022/9/29 上午9:55
from apps import openapi
from apps.core.simple_blueprint import SimpleBlueprint
from apps.helpers import make_json_response

bp = SimpleBlueprint("environ", __name__, url_prefix="/api/hmi/env")


@bp.route("/clear", methods=["GET", "POST"])
@openapi.api_doc(tags=["XLHB(标准库接口汇总)", "仿真环境"])
def clear():
    """清空仿真图环境.

    异常处理清空环境,HMI拆码垛异常处理中，点击清空环境按钮，会将托盘或者传送带上的纸箱清空
    """
    try:
        from xyz_env_manager.client import (
            get_planning_environment,
            clear_container_all_items,
            clear_planned_items,
        )
        from xyz_env_manager.client import clear_attached_collision_object

        workspace_env_msg = get_planning_environment()
        for workspace_msg in workspace_env_msg.workspaces:
            workspace_id = workspace_msg.workspace_id
            clear_container_all_items(workspace_id)
            clear_planned_items(workspace_id)
        clear_attached_collision_object("0", [], True)
    except ImportError as err:
        raise ImportError(
            "Import xyz_env_manager failed, please check if xyz_env_manager is installed"
        ) from err
    return make_json_response()
