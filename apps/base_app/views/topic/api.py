#  -*- coding: utf-8 -*-
#  Copyright (c) XYZ Robotics Inc. - All Rights Reserved
#  Unauthorized copying of this file, via any medium is strictly prohibited
#  Proprietary and confidential
#  Author: Yuhang Wu <yuhang.wu@xyzrobotics.ai>, 2022/9/29 上午9:48
from apps import openapi
from apps.helpers import make_json_response
from apps.settings import settings
from apps.core.simple_blueprint import SimpleBlueprint

bp = SimpleBlueprint("topic", __name__, url_prefix="/api/hmi/topic")


@bp.route("/", methods=["GET", "POST"])
@openapi.api_doc(tags=["XLHB(标准库接口汇总)", "仿真环境"])
def get_topic_list():
    """获取rviz的topic列表

    Returns:
        ---
        {
            "code": 0,
            "msg": "success",
            "data": [
                {
                    "topic": "/real_planning_environment_marker_topic",
                    "type": "visualization_msgs/MarkerArray"
                },
                {
                    "topic": "/booked_items_marker_topic",
                    "type": "visualization_msgs/MarkerArray"
                },
                {
                    "topic": "/tool_marker_robot_topic",
                    "type": "visualization_msgs/MarkerArray"
                },
             ]
         }
        ---
    """
    data = []
    for topic in settings.TOPIC_LIST:
        topic_describe = {
            "topic": topic,
            "type": "visualization_msgs/MarkerArray",
        }
        data.append(topic_describe)
    return make_json_response(data=data)
