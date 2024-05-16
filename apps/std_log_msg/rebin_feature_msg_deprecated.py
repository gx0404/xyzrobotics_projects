#-*- coding: utf-8 -*-

## batch related info
wait_batch_from_wms = {
    "msg_type": "info",
    "code": "I0044",
    "en_msg": "Wait batch info from WMS.",
    "zh_msg": "等待WMS的波次"
}
batch_received_from_wms = {
    "msg_type": "info",
    "code": "I0045",
    "en_msg": "Batch received from WMS.",
    "zh_msg": "接收到WMS的波次"
}

## shelf related info
wait_shelf_info = {
    "msg_type": "info",
    "code": "I0046",
    "en_msg": "Wait for shelf info.",
    "zh_msg": "等待HMI货架信息"
}
shelf_info_received = {
    "msg_type": "info",
    "code": "I0047",
    "en_msg": "Shelf info received.",
    "zh_msg": "接收到HMI货架信息"
}

## order related info
wait_batch_from_wcs = {
    "msg_type": "info",
    "code": "I0048",
    "en_msg": "Wait batch info from WCS.",
    "zh_msg": "等待WCS的波次信息"
}
batch_received_from_wcs = {
    "msg_type": "info",
    "code": "I0049",
    "en_msg": "Batch received from WCS.",
    "zh_msg": "接收到WCS的波次信息"
}

## tote scanning error (E004*)
tote_barcode_scanning_error = {
    "msg_type": "error",
    "code": "E0040",
    "en_msg": "Scanning tote barcode exceeds the timeout. Need to restart the scanner node.",
    "zh_msg": "扫描料箱码超时",
    "zh_tip": "确认箱子对着扫码枪一侧是否有条码，如有则需要重启扫码枪节点"
}
tote_batch_mismatch = {
    "msg_type": "error",
    "code": "E0041",
    "en_msg": "The tote does not belong to current batch.",
    "zh_msg": "料箱不属于该波次",
    "zh_tip": "请更换料箱或确认波次是否正确"
}

## cup related error (cup, the tool to realize picking with suction method)
pressure_error = {
    "msg_type": "error",
    "code": "E0001",
    "en_msg": "Cup is blocked.",
    "zh_msg": "吸盘被堵住",
    "zh_tip": "检查吸盘是否被堵住"
}
cup_dropped = {
    "msg_type": "error",
    "code": "E0002",
    "en_msg": "Cup is dropped.",
    "zh_msg": "吸盘掉落",
    "zh_tip": "检查吸盘以及快换工具是否掉落"
}
tool_lower_part_dropped = {
    "msg_type": "error",
    "code": "E0005",
    "en_msg": "Tool changer lower part dropped.",
    "zh_msg": "吸盘工具底部掉落",
    "zh_tip": "检查机器人工具末端是否掉落"
}

# hoist error (PLC error: E002*)
plc_ready_exceeds_timeout = {
    "msg_type": "error",
    "code": "E0024",
    "en_msg": "Waiting PLC ready exceeds the timeout.",
    "zh_msg": "PLC就位超时",
    "zh_tip": "请联系电气工程师"
}
hoist_ready_exceeds_timeout = {
    "msg_type": "error",
    "code": "E0025",
    "en_msg": "Waiting hoist ready exceeds the timeout.",
    "zh_msg": "提升机复位超时",
    "zh_tip": "查看PLC屏幕是否有报警信息,拍照记录,联系电气工程师"
}
sku_presence_before_first_action = {
    "msg_type": "error",
    "code": "E0026",
    "en_msg": "Something in the hoist funnel before the first action.",
    "zh_msg": "开始前提升机未清空",
    "zh_tip": "请清空提升机"
}
slip_into_order_bin_failure = {
    "msg_type": "error",
    "code": "E0027",
    "en_msg": "SKU does not slip into the order bin after several tries of flipping.",
    "zh_msg": "sku滑入订单箱超时",
    "zh_tip": "检查是否sku太大被卡住或者有粘性粘在提升机上,若都不是,继续重试,若多次连续发生同样问题,联系XYZ研发人员"
}
release_tote_exceeds_timeout = {
    "msg_type": "error",
    "code": "E0028",
    "en_msg": "Releasing tote exceeds the timeout.",
    "zh_msg": "释放料箱超时",
    "zh_tip": "查看PLC屏幕是否有报警信息,拍照记录,联系电气工程师"
}

## not severe errors
batch_error = {
    "msg_type": "error",
    "code": "E0060",
    "en_msg": "No batch info.",
    "zh_msg": "没有波次信息",
    "zh_tip": "请确认是否有波次,若无,请联系贵方WMS研发人员,若有,请联系XYZ研发人员"
}

## shelf related
shelf_barcode_scanning_error = {
    "msg_type": "error",
    "code": "E0100",
    "en_msg": "Shelf barcode scanning exceeds the timeout!",
    "zh_msg": "货架码扫描超时",
    "zh_tip": "检查货架条码是否在正确位置,重启扫码枪节点后重新开始,若仍有同样问题,检查货架附近扫码枪是否工作（有灯闪烁）,记录状态,联系研发"
}
shelf_bound_success = {
    "msg_type": "info",
    "code": "I0100",
    "en_msg": "Shelf bound to WMS success.",
    "zh_msg": "货架绑定到WMS成功"
}
shelf_bound_failure = {
    "msg_type": "error",
    "code": "E0101",
    "en_msg": "Shelf bound to WMS failure. Please change the shelf.",
    "zh_msg": "货架绑定WMS失败",
    "zh_tip": "检查人工分播系统中是否该货架已处于绑定状态,若是,先解绑货架再点击开始重试;若不是,联系贵方WMS研发人员解决"
}
all_shelves_occupied = {
    "msg_type": "error",
    "code": "E0102",
    "en_msg": "All shelves are occupied. Please unbind and change a new shelf.",
    "zh_msg": "没有空闲货架",
    "zh_tip": "检查人工分播处货架绑定状态,若有非正常绑定状态,先解绑后开始重试,若仍有问题,联系XYZ研发人员"
}
release_shelf_received = {
    "msg_type": "info",
    "code": "I0102",
    "en_msg": "Releasing shelf signal is received.",
    "zh_msg": "解绑货架信号已接收"
}
release_shelf_success = {
    "msg_type": "info",
    "code": "I0103",
    "en_msg": "Release shelf success.",
    "zh_msg": "解绑货架成功"
}
release_shelf_failure = {
    "msg_type": "info",
    "code": "I0104",
    "en_msg": "Release shelf failure.",
    "zh_msg": "解绑货架失败"
}

## send tote info related
batch_received_success = {
    "msg_type": "info",
    "code": "I0110",
    "en_msg": "Batch received success. Robot will move.",
    "zh_msg": "波次接收成功,机器人将开始工作"
}
invalid_tote_info_error_code = {
    "msg_type": "error",
    "code": "E0110",
    "en_msg": "Invalid error code for tote info.",
    "zh_msg": "无效的料箱的错误码",
    "zh_tip": "请联系XYZ研发人员"
}

wms_failure = {
    "msg_type": "error",
    "code": "E0111",
    "en_msg": "WMS returns FAILURE. Please take away this tote and contact WMS R&D!",
    "zh_msg": "WMS返回失败",
    "zh_tip": "联系贵方WMS研发人员处理,换新料箱继续"
}
order_num_exceeds_limit = {
    "msg_type": "error",
    "code": "E0112",
    "en_msg": "Order num in the current tote exceeds the maximal limit. Please change a new tote.",
    "zh_msg": "订单数量超限",
    "zh_tip": "取走该料箱拿去人工复核,换新料箱继续"
}
bin_num_insufficient = {
    "msg_type": "error",
    "code": "E0113",
    "en_msg": "Available bin number is insufficient. Please change a new shelf.",
    "zh_msg": "货架可用格口数不足",
    "zh_tip": "检查人工分播处货架绑定状态,若有空闲货架被绑定,先解绑,再重新开始重试,若仍有相同问题,联系贵方WMS研发人员处理"
}
recycle_bin_insufficient = {
    "msg_type": "error",
    "code": "E0114",
    "en_msg": "Available recycle bin number is insufficient.",
    "zh_msg": "废料口数量不足",
    "zh_tip": "查看人工分播处货架绑定状态,若有不正常绑定状态,先解绑再开始重试,若仍有问题,联系贵方WMS研发人员处理"
}
wms_error = {
    "msg_type": "error",
    "code": "E0119",
    "en_msg": "WMS ERROR! Request human intervention!",
    "zh_msg": "WMS错误",
    "zh_tip": "请联系贵方WMS研发人员"
}

order_bin_info_error = {
    "msg_type": "error",
    "code": "E0120",
    "en_msg": "Error in getting order bin info.",
    "zh_msg": "获取订单格口信息时错误",
    "zh_tip": "联系XYZ研发人员和贵方研发人员共同解决"
}
update_error = {
    "msg_type": "error",
    "code": "E0125",
    "en_msg": "Error in updating order bin info.",
    "zh_msg": "格口更新错误",
    "zh_tip": "联系XYZ研发人员和贵方研发人员共同解决"
}

no_cup_for_topple = {
    "msg_type": "error",
    "code": "E0077",
    "en_msg": "No cup is collision free for topple action.",
    "zh_msg": "搅拌时可能发生碰撞",
    "zh_tip": "请释放料箱,手动播种"
}