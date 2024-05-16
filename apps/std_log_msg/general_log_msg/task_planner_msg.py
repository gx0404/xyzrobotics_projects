#-*- coding: utf-8 -*-

"""
info msg
"""


no_pick_point = {
    "msg_type": "info",
    "code": "I0015",
    "en_msg": "Tote is not empty but there is no primitive.",
    "zh_msg": "料箱未清空"
}
tote_empty = {
    "msg_type": "info",
    "code": "I0016",
    "en_msg": "Tote is empty.",
    "zh_msg": "料箱已空"
}
double_pick = {
    "msg_type": "info",
    "code": "I0020",
    "en_msg": "Double pick detected by active vision.",
    "zh_msg": "active vision检测到同时吸取到多个SKU"
}
object_dropped = {
    "msg_type": "info",
    "code": "I0021",
    "en_msg": "Object dropped.",
    "zh_msg": "SKU掉落"
}
potential_double_pick = {
    "msg_type": "info",
    "code": "I0022",
    "en_msg": "Potential double picking detected by place vision.",
    "zh_msg": "place vision检测到可能同时吸取到多个SKU"
}

"""
warning msg
"""
object_dropped_warning = {
    "msg_type": "warning",
    "code": "W0002",
    "en_msg": "Object dropped.",
    "zh_msg": "SKU掉落",
    "zh_tip": "请在机器播种结束后,进行补播种",
    "timeout": None
}
call_remote_helper = {
    "msg_type": "warning",
    "code": "W0003",
    "en_msg": "Need to call remote helper.",
    "zh_msg": "需要人工帮助",
    "zh_tip": "请完成物体标注，或者回报当前状态",
    "timeout": 60
}
no_pick_point_warning = {
    "msg_type": "warning",
    "code": "W0004",
    "en_msg": "Tote is not empty but there is no primitive.",
    "zh_msg": "料箱未清空",
    "zh_tip": "退箱后请手动播种",
    "timeout": None
}
tote_empty_warning = {
    "msg_type": "warning",
    "code": "W0005",
    "en_msg": "Tote is empty.",
    "zh_msg": "料箱已空",
    "zh_tip": "请释放料箱",
    "timeout": None
}
consecutive_failure = {
    "msg_type": "warning",
    "code": "W0006",
    "en_msg": "Stop due to failure too many times. Tote may not be empty. Please check.",
    "zh_msg": "连续吸取失败多次退箱",
    "zh_tip": "请手动完成补播种，并检查吸盘状态是否异常",
    "timeout": 100000
}

"""
error msg
"""
general_error = {
    "msg_type": "error",
    "code": "E9999",
    "en_msg": "General error.",
    "zh_msg": "未定义错误",
    "zh_tip": "请联系XYZ研发人员"
}