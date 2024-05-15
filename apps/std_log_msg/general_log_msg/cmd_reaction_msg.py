#-*- coding: utf-8 -*-

start_success = {
    "msg_type": "info",
    "code": "I0001",
    "en_msg": "Start success.",
    "zh_msg": "开始成功"
}

start_failure = {
    "msg_type": "info",
    "code": "I0002",
    "en_msg": "Start failure.",
    "zh_msg": "开始失败"
}

restart_success = {
    "msg_type": "info",
    "code": "I0003",
    "en_msg": "Restart success.",
    "zh_msg": "继续成功"
}

restart_failure = {
    "msg_type": "info",
    "code": "I0004",
    "en_msg": "Restart failure.",
    "zh_msg": "继续失败"
}

stop_success = {
    "msg_type": "info",
    "code": "I0005",
    "en_msg": "Cycle stop success.",
    "zh_msg": "停止成功"
}

stop_failure = {
    "msg_type": "info",
    "code": "I0006",
    "en_msg": "Cycle stop failure.",
    "zh_msg": "停止失败"
}

pause_success = {
    "msg_type": "info",
    "code": "I0007",
    "en_msg": "Cycle pause success.",
    "zh_msg": "暂停成功"
}

pause_failure = {
    "msg_type": "info",
    "code": "I0008",
    "en_msg": "Cycle pause failure.",
    "zh_msg": "暂停失败"
}


release_tote_success = {
    "msg_type": "info",
    "code": "I0010",
    "en_msg": "Release tote success.",
    "zh_msg": "释放料箱成功"
}

release_tote_failure = {
    "msg_type": "info",
    "code": "I0011",
    "en_msg": "Release tote failure.",
    "zh_msg": "释放料箱失败"
}


invalid_start_signal = {
    "msg_type": "info",
    "code": "I1000",
    "en_msg": "Invalid start signal.",
    "zh_msg": "无效的开始信号"
}

start_signal_received = {
    "msg_type": "info",
    "code": "I1001",
    "en_msg": "Start signal received.",
    "zh_msg": "收到开始信号"
}
start_signal_rejected = {
    "msg_type": "info",
    "code": "I1002",
    "en_msg": "Start signal rejected.",
    "zh_msg": "开始信号被拒绝"
}

stop_signal_received = {
    "msg_type": "info",
    "code": "I1005",
    "en_msg": "Stop signal received.",
    "zh_msg": "收到停止信号"
}

release_tote_received = {
    "msg_type": "info",
    "code": "I1007",
    "en_msg": "Release tote signal received.",
    "zh_msg": "收到释放料箱信号"
}
release_tote_rejected = {
    "msg_type": "info",
    "code": "I1008",
    "en_msg": "Release tote signal rejected.",
    "zh_msg": "释放料箱信号被拒绝"
}
start_release_tote = {
    "msg_type": "info",
    "code": "I1009",
    "en_msg": "Start releasing tote.",
    "zh_msg": "开始释放料箱"
}

init_pick_tool_error = {
    "msg_type": "error",
    "code": "E0004",
    "en_msg": "The number of cups on the rack is incorrect. Please check the rack and make sure there is only one cup on the gripper.",
    "zh_msg": "在槽位上的吸盘数量错误",
    "zh_tip": "请检查吸盘槽位,以及是否只有一个吸盘在机械臂上"
}