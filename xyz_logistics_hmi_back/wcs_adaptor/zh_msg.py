# -*- coding:utf-8 -*-
from collections import UserDict

from apps import settings

# 一旦接收到下方集合中的error_code, 则将向WCS发送异常报告.
# 以下のセットで error_code を受信すると、例外レポートが WCS に送信される。
# https://xyz-robotics.atlassian.net/browse/MQWR-770
REPORT_EXCEPTION_CODES = {"E0608"}


class _AllError(UserDict):
    """异常消息，通过异常码获取对应的异常消息.

    使用 `.get()` 方法可以获取异常消息，获取到的消息会自动根据语言转换，例如：

    默认系统为中文，触发 `99999` 异常，将返回：
    {
        "code": "99999"
        "msg": "未定义异常",
        "tip": "请联系售后人员",
        ...
    }

    当语言切换为日语时，触发 `99999` 异常，则将返回如下异常消息：
    {
        "code": "99999"
        "msg": "未定義のエラー",
        "tip": "アフターサービス担当までご連絡ください",
        ...
    }

    具体包含以下内容：
    {
        "code": code,                    # 异常码
        "error_msg": error["error_msg"], # 异常名
        "msg": msg,                      # 异常详情
        "tip": tip,                      # 操作建议
        "msg_type": "error",             # 异常等级
        "class": "motion",               # 异常来源
    }

    """

    def get(self, code, default=None):
        """根据系统语言重新构造异常信息字典.

        Args:
            code: 异常码
            default: 接收任意参数，当 code 不存时，返回 DEFAULT_ERROR

        Returns:
            dict: 异常信息
        """
        lang = settings.LANGUAGE.name
        code = str(code)

        def remake_error_info(error: dict):
            if lang == "zh":
                # stupid design
                tip = error["tip"]
            else:
                tip = error[f"{lang}_tip"]
            msg = error[f"{lang}_msg"]
            return {
                "code": code,
                "error_msg": error["error_msg"],
                "msg": msg,
                "tip": tip,
                "msg_type": "error",  # 异常等级
                "class": "motion",  # 异常来源
            }

        if code in self.data:
            return remake_error_info(self.data[code])
        elif default:
            return remake_error_info(DEFAULT_ERROR)
        else:
            raise KeyError(f"异常码{code}不存在")


ALL_ERROR = _AllError()

ALL_ERROR["10000"] = {
    "error_code": "10000",
    "error_msg": "all position num fault",
    "zh_msg": "拣配任务托盘虚拟储位匹配失败",
    "tip": "已自动回报WCS虚拟储位匹配失败",
    "ja_msg": "位置号匹配失败",
    "ja_tip": "已自动回报WCS虚拟储位匹配失败",
    "en_msg": "位置号匹配失败",
    "en_tip": "已自动回报WCS虚拟储位匹配失败",
    "de_msg": "位置号匹配失败",
    "de_tip": "已自动回报WCS虚拟储位匹配失败",
}


ALL_ERROR["10001"] = {
    "error_code": "10001",
    "error_msg": "depal tote vision fault",
    "zh_msg": "拣配任务料箱视觉识别异常",
    "tip": "摆正拣配托盘位置、料箱垛型或者压平料箱内超出的物料,或者稍微拉开料箱之间的距离后,在hmi初始化节点重新连接机器人机器人节点,然后点开始运行程序",
    "ja_msg": "位置号匹配失败",
    "ja_tip": "已自动回报WCS虚拟储位匹配失败",
    "en_msg": "位置号匹配失败",
    "en_tip": "已自动回报WCS虚拟储位匹配失败",
    "de_msg": "位置号匹配失败",
    "de_tip": "已自动回报WCS虚拟储位匹配失败",
}


ALL_ERROR["10002"] = {
    "error_code": "10002",
    "error_msg": "cage vision fault",
    "zh_msg": "笼车或者围栏视觉识别异常",
    "tip": "摆正笼车位置,围栏如果倾斜请摆正,在hmi初始化节点重新连接机器人节点,然后点开始运行程序",
    "ja_msg": "位置号匹配失败",
    "ja_tip": "已自动回报WCS虚拟储位匹配失败",
    "en_msg": "位置号匹配失败",
    "en_tip": "已自动回报WCS虚拟储位匹配失败",
    "de_msg": "位置号匹配失败",
    "de_tip": "已自动回报WCS虚拟储位匹配失败",
}



ALL_ERROR["10003"] = {
    "error_code": "10003",
    "error_msg": "calculate move_camera_pose fault ",
    "zh_msg": "拣配托盘计算下一次眼在手上拍照的位姿失败",
    "tip": "摆正拣配托盘位置、料箱位置,如果托盘存在高列和低列，高列倾斜,请稍微隔开,在hmi初始化节点,重新连接机器人节点，然后点开始运行程序",
    "ja_msg": "位置号匹配失败",
    "ja_tip": "已自动回报WCS虚拟储位匹配失败",
    "en_msg": "位置号匹配失败",
    "en_tip": "已自动回报WCS虚拟储位匹配失败",
    "de_msg": "位置号匹配失败",
    "de_tip": "已自动回报WCS虚拟储位匹配失败",
}

ALL_ERROR["10004"] = {
    "error_code": "10004",
    "error_msg": "calculate depal motion fault",
    "zh_msg": "计算拣配任务运动轨迹失败,检测到碰撞",
    "tip": "摆正拣配托盘位置、料箱位置,如果托盘存在高列和低列，高列倾斜,请稍微隔开,在hmi初始化节点,重新连接机器人节点，然后点开始运行程序",
    "ja_msg": "位置号匹配失败",
    "ja_tip": "已自动回报WCS虚拟储位匹配失败",
    "en_msg": "位置号匹配失败",
    "en_tip": "已自动回报WCS虚拟储位匹配失败",
    "de_msg": "位置号匹配失败",
    "de_tip": "已自动回报WCS虚拟储位匹配失败",
}

ALL_ERROR["10005"] = {
    "error_code": "10005",
    "error_msg": "cache tote vision fault",
    "zh_msg": "缓存区视觉报错",
    "tip": "摆正缓存区托盘位置、料箱位置,或者压平料箱内超出的物料,或者稍微拉开料箱之间的距离,在hmi初始化节点,重新连接机器人节点，然后点开始运行程序",
    "ja_msg": "位置号匹配失败",
    "ja_tip": "已自动回报WCS虚拟储位匹配失败",
    "en_msg": "位置号匹配失败",
    "en_tip": "已自动回报WCS虚拟储位匹配失败",
    "de_msg": "位置号匹配失败",
    "de_tip": "已自动回报WCS虚拟储位匹配失败",
}


ALL_ERROR["10006"] = {
    "error_code": "10006",
    "error_msg": "calculate depal motion fault",
    "zh_msg": "拣配任务还原回来时,计算运动轨迹失败,检测到碰撞",
    "tip": "摆正拣配托盘位置、料箱位置,如果托盘存在高列和低列，高列倾斜,请稍微隔开,在hmi初始化节点,重新连接机器人节点，然后点开始运行程序",
    "ja_msg": "位置号匹配失败",
    "ja_tip": "已自动回报WCS虚拟储位匹配失败",
    "en_msg": "位置号匹配失败",
    "en_tip": "已自动回报WCS虚拟储位匹配失败",
    "de_msg": "位置号匹配失败",
    "de_tip": "已自动回报WCS虚拟储位匹配失败",
}


ALL_ERROR["10007"] = {
    "error_code": "10007",
    "error_msg": "merge tote vision fault",
    "zh_msg": "合托任务主托盘(拣配任务来料托盘)料箱视觉识别异常",
    "tip": "摆正主托盘(拣配任务来料托盘)位置、料箱垛型或者压平料箱内超出的物料,或者稍微拉开料箱之间的距离后,在hmi初始化节点重新连接机器人机器人节点,然后点开始运行程序",
    "ja_msg": "位置号匹配失败",
    "ja_tip": "已自动回报WCS虚拟储位匹配失败",
    "en_msg": "位置号匹配失败",
    "en_tip": "已自动回报WCS虚拟储位匹配失败",
    "de_msg": "位置号匹配失败",
    "de_tip": "已自动回报WCS虚拟储位匹配失败",
}

ALL_ERROR["10008"] = {
    "error_code": "10008",
    "error_msg": "merge tote vision fault",
    "zh_msg": "合托任务次托盘(拣配任务缓存托盘)料箱视觉识别异常",
    "tip": "摆正次托盘(拣配任务缓存托盘)位置、料箱垛型或者压平料箱内超出的物料,或者稍微拉开料箱之间的距离后,在hmi初始化节点重新连接机器人机器人节点,然后点开始运行程序",
    "ja_msg": "位置号匹配失败",
    "ja_tip": "已自动回报WCS虚拟储位匹配失败",
    "en_msg": "位置号匹配失败",
    "en_tip": "已自动回报WCS虚拟储位匹配失败",
    "de_msg": "位置号匹配失败",
    "de_tip": "已自动回报WCS虚拟储位匹配失败",
}


ALL_ERROR["10009"] = {
    "error_code": "10009",
    "error_msg": "calculate merge motion fault",
    "zh_msg": "合托任务计算从主托盘(拣配任务来料托盘)到笼车托盘轨迹失败",
    "tip": "摆正合托任务主托盘(拣配任务来料托盘)托盘位置、料箱位置,如果托盘存在高列和低列，高列倾斜,请稍微隔开,在hmi初始化节点,重新连接机器人节点，然后点开始运行程序,如果还失败联系XYZ人员",
    "ja_msg": "位置号匹配失败",
    "ja_tip": "已自动回报WCS虚拟储位匹配失败",
    "en_msg": "位置号匹配失败",
    "en_tip": "已自动回报WCS虚拟储位匹配失败",
    "de_msg": "位置号匹配失败",
    "de_tip": "已自动回报WCS虚拟储位匹配失败",
}

ALL_ERROR["10010"] = {
    "error_code": "10010",
    "error_msg": "calculate merge motion fault",
    "zh_msg": "合托任务计算从次托盘(拣配任务缓存托盘)到主托盘(拣配任务来料托盘)轨迹失败",
    "tip": "摆正合托任务主托盘(拣配任务来料托盘)和次托盘(拣配任务缓存托盘)的托盘位置、料箱位置,如果托盘存在高列和低列，高列倾斜,请稍微隔开,在hmi初始化节点,重新连接机器人节点，然后点开始运行程序,如果还失败联系XYZ人员",
    "ja_msg": "位置号匹配失败",
    "ja_tip": "已自动回报WCS虚拟储位匹配失败",
    "en_msg": "位置号匹配失败",
    "en_tip": "已自动回报WCS虚拟储位匹配失败",
    "de_msg": "位置号匹配失败",
    "de_tip": "已自动回报WCS虚拟储位匹配失败",
}


ALL_ERROR["10011"] = {
    "error_code": "10011",
    "error_msg": "calculate merge motion fault",
    "zh_msg": "合托任务计算从笼车托盘到主托盘(拣配任务来料托盘)轨迹失败",
    "tip": "摆正合托任务主托盘(拣配任务来料托盘)托盘位置、料箱位置,如果托盘存在高列和低列，高列倾斜,请稍微隔开,在hmi初始化节点,重新连接机器人节点，然后点开始运行程序,如果还失败联系XYZ人员",
    "ja_msg": "位置号匹配失败",
    "ja_tip": "已自动回报WCS虚拟储位匹配失败",
    "en_msg": "位置号匹配失败",
    "en_tip": "已自动回报WCS虚拟储位匹配失败",
    "de_msg": "位置号匹配失败",
    "de_tip": "已自动回报WCS虚拟储位匹配失败",
}


ALL_ERROR["10012"] = {
    "error_code": "10012",
    "error_msg": "calculate depal pallet resume motion fault",
    "zh_msg": "拣配任务,拣配托盘续码匹配失败",
    "tip": "摆正拣配托盘位置、料箱位置,如果托盘存在高列和低列，高列倾斜,请稍微隔开,在hmi初始化节点,重新连接机器人节点，然后点开始运行程序,如果还失败联系XYZ人员",
    "ja_msg": "位置号匹配失败",
    "ja_tip": "已自动回报WCS虚拟储位匹配失败",
    "en_msg": "位置号匹配失败",
    "en_tip": "已自动回报WCS虚拟储位匹配失败",
    "de_msg": "位置号匹配失败",
    "de_tip": "已自动回报WCS虚拟储位匹配失败",
}

ALL_ERROR["10013"] = {
    "error_code": "10013",
    "error_msg": "vision service fault",
    "zh_msg": "MAX视觉启动失败",
    "tip": "重新启动任务流程节点,后续再启动XYZ-MAX视觉",
    "ja_msg": "位置号匹配失败",
    "ja_tip": "已自动回报WCS虚拟储位匹配失败",
    "en_msg": "位置号匹配失败",
    "en_tip": "已自动回报WCS虚拟储位匹配失败",
    "de_msg": "位置号匹配失败",
    "de_tip": "已自动回报WCS虚拟储位匹配失败",
}

ALL_ERROR["10014"] = {
    "error_code": "10014",
    "error_msg": "move camera vision fault",
    "zh_msg": "拣配任务机械臂相机视觉识别失败",
    "tip": "查看机械臂夹具上相机正下方的料箱,压平料箱内超出的物料,或者稍微拉开正下方料箱之间的距离,物料然后在hmi初始化节点,重新连接机器人节点，然后点开始运行程序",
    "ja_msg": "位置号匹配失败",
    "ja_tip": "已自动回报WCS虚拟储位匹配失败",
    "en_msg": "位置号匹配失败",
    "en_tip": "已自动回报WCS虚拟储位匹配失败",
    "de_msg": "位置号匹配失败",
    "de_tip": "已自动回报WCS虚拟储位匹配失败",
}

ALL_ERROR["10015"] = {
    "error_code": "10015",
    "error_msg": "pal tote motion plan fault",
    "zh_msg": "笼车空箱回收,笼车-输送线扫码运动规划失败",
    "tip": "检查笼车定位,摆正后在hmi初始化节点,重新连接机器人节点，然后点开始运行程序",
    "ja_msg": "位置号匹配失败",
    "ja_tip": "已自动回报WCS虚拟储位匹配失败",
    "en_msg": "位置号匹配失败",
    "en_tip": "已自动回报WCS虚拟储位匹配失败",
    "de_msg": "位置号匹配失败",
    "de_tip": "已自动回报WCS虚拟储位匹配失败",
}

ALL_ERROR["10016"] = {
    "error_code": "10016",
    "error_msg": "init tool io fault",
    "zh_msg": "初始化运行回原位时，夹具自检未通过",
    "tip": "检查夹具是否抓取物料,夹具气缸IO信号是否正常,请将机器人夹具恢复，机器人回到原位,重新连接机器人节点，然后点开始运行程序",
    "ja_msg": "位置号匹配失败",
    "ja_tip": "已自动回报WCS虚拟储位匹配失败",
    "en_msg": "位置号匹配失败",
    "en_tip": "已自动回报WCS虚拟储位匹配失败",
    "de_msg": "位置号匹配失败",
    "de_tip": "已自动回报WCS虚拟储位匹配失败",
}

ALL_ERROR["10017"] = {
    "error_code": "10017",
    "error_msg": "restore depal calculate all path fault",
    "zh_msg": "拣配还原所有路径搜索失败,无法还原",
    "tip": "观察拣配托盘，摆正拣配托盘位置、料箱位置，查看是否存在高列和低列，高列倾斜，请稍微隔开,然后在hmi初始化节点,重新连接机器人节点，然后点开始运行程序",
    "ja_msg": "位置号匹配失败",
    "ja_tip": "已自动回报WCS虚拟储位匹配失败",
    "en_msg": "位置号匹配失败",
    "en_tip": "已自动回报WCS虚拟储位匹配失败",
    "de_msg": "位置号匹配失败",
    "de_tip": "已自动回报WCS虚拟储位匹配失败",
}

ALL_ERROR["10018"] = {
    "error_code": "10018",
    "error_msg": "scan code fault",
    "zh_msg": "笼车扫码异常，条码物料对不上",
    "tip": "请检查输送线上料箱条码是否正确，手动完成笼车任务，并且结束当前任务",
    "ja_msg": "位置号匹配失败",
    "ja_tip": "已自动回报WCS虚拟储位匹配失败",
    "en_msg": "位置号匹配失败",
    "en_tip": "已自动回报WCS虚拟储位匹配失败",
    "de_msg": "位置号匹配失败",
    "de_tip": "已自动回报WCS虚拟储位匹配失败",
}


ALL_ERROR["10019"] = {
    "error_code": "10019",
    "error_msg": "scan code fault",
    "zh_msg": "笼车空箱回收影像异常,无法获取空箱信息,重新连接机器人节点，然后点开始运行程序",
    "tip": "请检查输送线以及JCS系统,将料箱放回到原先笼车,",
    "ja_msg": "位置号匹配失败",
    "ja_tip": "已自动回报WCS虚拟储位匹配失败",
    "en_msg": "位置号匹配失败",
    "en_tip": "已自动回报WCS虚拟储位匹配失败",
    "de_msg": "位置号匹配失败",
    "de_tip": "已自动回报WCS虚拟储位匹配失败",
}



ALL_ERROR["10020"] = {
    "error_code": "10020",
    "error_msg": "拣配任务中欧到笼车,放置转向180度运动规划失败",
    "zh_msg": "拣配任务中欧到笼车,放置转向180度运动规划失败",
    "tip": "摆正拣配托盘位置、料箱位置,如果托盘存在高列和低列，高列倾斜,请稍微隔开,在hmi初始化节点,重新连接机器人节点，然后点开始运行程序",
    "ja_msg": "位置号匹配失败",
    "ja_tip": "已自动回报WCS虚拟储位匹配失败",
    "en_msg": "位置号匹配失败",
    "en_tip": "已自动回报WCS虚拟储位匹配失败",
    "de_msg": "位置号匹配失败",
    "de_tip": "已自动回报WCS虚拟储位匹配失败",
}


ALL_ERROR["20000"] = {
    "error_code": "20000",
    "error_msg": "NoValidObjects",
    "zh_msg": "无抓取位姿",
    "tip": "人工通过HMI上的local helper功能进行辅助标注",
    "ja_msg": "つかみ取る可能姿勢なし",
    "ja_tip": "HMIのローカルヘルパー機能による手動マーキング",
    "en_msg": "No grasping posture",
    "en_tip": "Manually perform auxiliary labeling through the local helper function on the HMI",
    "de_msg": "Keine Griffpose ",
    "de_tip": "Das manuelle Kennzeichen durch lokale Hilfsfunktion auf der HMI ",
}
ALL_ERROR["20001"] = {
    "error_code": "20001",
    "error_msg": "InitFailed",
    "zh_msg": "视觉初始化失败",
    "tip": "请联系售后人员",
    "ja_msg": "ビジュアル初期化失敗",
    "ja_tip": "アフターサービス担当までご連絡ください",
    "en_msg": "Visual initialization failed",
    "en_tip": "Please contact the after-sales staff",
    "de_msg": "Initialisierung des Bildverarbeitungssystems fehlgeschlagen ",
    "de_tip": "Bitte wenden Sie sich an das Kundendienstpersonal",
}
ALL_ERROR["20002"] = {
    "error_code": "20002",
    "error_msg": "InvalidCmd",
    "zh_msg": "错误的指令",
    "tip": "请联系售后人员",
    "ja_msg": "間違った指示",
    "ja_tip": "アフターサービス担当までご連絡ください",
    "en_msg": "Wrong instruction",
    "en_tip": "Please contact the after-sales staff",
    "de_msg": "Falsche Anweisung",
    "de_tip": "Bitte wenden Sie sich an das Kundendienstpersonal",
}
ALL_ERROR["20003"] = {
    "error_code": "20003",
    "error_msg": "InvalidToteId",
    "zh_msg": "无效的tote id",
    "tip": "请联系售后人员",
    "ja_msg": "無効なカメラtote id",
    "ja_tip": "アフターサービス担当までご連絡ください",
    "en_msg": "Invalid tote id",
    "en_tip": "Please contact the after-sales staff",
    "de_msg": "Ungültige Tote-ID",
    "de_tip": "Bitte wenden Sie sich an das Kundendienstpersonal",
}
ALL_ERROR["20004"] = {
    "error_code": "20004",
    "error_msg": "InvalidSensorId",
    "zh_msg": "无效的相机id",
    "tip": "请联系售后人员",
    "ja_msg": "無効id",
    "ja_tip": "アフターサービス担当までご連絡ください",
    "en_msg": "Invalid camera id",
    "en_tip": "Please contact the after-sales staff",
    "de_msg": "Ungültige Kamera-ID",
    "de_tip": "Bitte wenden Sie sich an das Kundendienstpersonal",
}
ALL_ERROR["20005"] = {
    "error_code": "20005",
    "error_msg": "ToteHasNoSensor",
    "zh_msg": "tote没有相机",
    "tip": "请联系售后人员",
    "ja_msg": "トートはカメラにバインドされていません",
    "ja_tip": "アフターサービス担当までご連絡ください",
    "en_msg": "Tote is not bound to the camera",
    "en_tip": "Please contact the after-sales staff",
    "de_msg": "Tote hat keine Kamera",
    "de_tip": "Bitte wenden Sie sich an das Kundendienstpersonal",
}
ALL_ERROR["20006"] = {
    "error_code": "20006",
    "error_msg": "GetSafeHeightFailed",
    "zh_msg": "视觉获取顶层高度失败",
    "tip": "请联系售后人员",
    "ja_msg": "安全な高さの取得に失敗",
    "ja_tip": "アフターサービス担当までご連絡ください",
    "en_msg": "Failed to get top-level height visually",
    "en_tip": "Please contact the after-sales staff",
    "de_msg": "Visualle Messung der oberen Höhe nicht möglich",
    "de_tip": "Bitte wenden Sie sich an das Kundendienstpersonal",
}
ALL_ERROR["20007"] = {
    "error_code": "20007",
    "error_msg": " Double pick detected",
    "zh_msg": "视觉检测到机械手出现多抓",
    "tip": "请联系售后人员",
    "ja_msg": "ダブルピック検出",
    "ja_tip": "アフターサービス担当までご連絡ください",
    "en_msg": "Visual detection of multi-grabbing of the manipulator",
    "en_tip": "Please contact the after-sales staff",
    "de_msg": "Mehrere Griffe der Roboter ist erkennt durch Bildverarbeitungssystem ",
    "de_tip": "Bitte wenden Sie sich an das Kundendienstpersonal",
}
ALL_ERROR["20010"] = {
    "error_code": "20010",
    "error_msg": "OpenCameraFailed",
    "zh_msg": "打开相机失败",
    "tip": "请断开相机开关电源，重新插拔相机网线，重启相机，再进行测试，如未恢复，请联系售后人员",
    "ja_msg": "カメラ起動失敗",
    "ja_tip": "カメラの電源を切り、ネットワークケーブルを挿し直して、カメラを再起動し、再度テストしてください。復元されない場合，アフターサービス担当までご連絡ください",
    "en_msg": "Failed to open camera",
    "en_tip": "Please disconnect the power switch of the camera, re-plug the camera network cable, restart the camera, and test again. If it does not recover, please contact the after-sales personnel.",
    "de_msg": "Kamera kann nicht geöffnet werden",
    "de_tip": "Bitte unterbrechen Sie die Stromversorgung der Kamera, stecken Sie das Netzwerkkabel der Kamera wieder ein, starten Sie die Kamera neu und testen Sie sie dann erneut. Wenn das Problem nicht behoben werden kann, wenden Sie sich bitte an den Kundendienst.",
}
ALL_ERROR["20011"] = {
    "error_code": "20011",
    "error_msg": "CaptureImagesFailed",
    "zh_msg": "相机拍照失败",
    "tip": "请断开相机开关电源，重新插拔相机网线，重启相机，再进行测试，如未恢复，请联系售后人员",
    "ja_msg": "カメラ写真撮影失敗",
    "ja_tip": "カメラの電源を切り、ネットワークケーブルを挿し直して、カメラを再起動し、再度テストしてください。復元されない場合，アフターサービス担当までご連絡ください",
    "en_msg": "The camera failed to take a picture",
    "en_tip": "Please disconnect the power switch of the camera, re-plug the camera network cable, restart the camera, and test again. If it does not recover, please contact the after-sales personnel.",
    "de_msg": "Die Kamera kann keine Bilder aufnehmen",
    "de_tip": "Bitte unterbrechen Sie die Stromversorgung der Kamera, stecken Sie das Netzwerkkabel der Kamera wieder ein, starten Sie die Kamera neu und testen Sie sie dann erneut. Wenn das Problem nicht behoben werden kann, wenden Sie sich bitte an den Kundendienst.",
}
ALL_ERROR["20012"] = {
    "error_code": "20012",
    "error_msg": "CameraBlocked",
    "zh_msg": "相机阻塞",
    "tip": "请断开相机开关电源，重新插拔相机网线，重启相机，再进行测试，如未恢复，请联系售后人员",
    "ja_msg": "カメラがブロックされた",
    "ja_tip": "カメラの電源を切り、ネットワークケーブルを挿し直して、カメラを再起動し、再度テストしてください。復元されない場合，アフターサービス担当までご連絡ください",
    "en_msg": "Camera blocked",
    "en_tip": "Please disconnect the power switch of the camera, re-plug the camera network cable, restart the camera, and test again. If it does not recover, please contact the after-sales personnel.",
    "de_msg": "Kamera blockiert",
    "de_tip": "Bitte unterbrechen Sie die Stromversorgung der Kamera, stecken Sie das Netzwerkkabel der Kamera wieder ein, starten Sie die Kamera neu und testen Sie sie dann erneut. Wenn das Problem nicht behoben werden kann, wenden Sie sich bitte an den Kundendienst.",
}
ALL_ERROR["20013"] = {
    "error_code": "20013",
    "error_msg": "CameraMoved",
    "zh_msg": "相机被移动过",
    "tip": "请联系售后人员",
    "ja_msg": "カメラが移動された",
    "ja_tip": "アフターサービス担当までご連絡ください",
    "en_msg": "Camera was moved",
    "en_tip": "Please contact the after-sales staff",
    "de_msg": "Kamera wurde bewegt",
    "de_tip": "Bitte wenden Sie sich an das Kundendienstpersonal",
}
ALL_ERROR["20020"] = {
    "error_code": "20020",
    "error_msg": "NoValidSnapshot",
    "zh_msg": "无效的图像源",
    "tip": "请联系售后人员",
    "ja_msg": "無効な画像ソース",
    "ja_tip": "アフターサービス担当までご連絡ください",
    "en_msg": "Invalid image source",
    "en_tip": "Please contact the after-sales staff",
    "de_msg": "Ungültige Bildquelle",
    "de_tip": "Bitte wenden Sie sich an das Kundendienstpersonal",
}
ALL_ERROR["20021"] = {
    "error_code": "20021",
    "error_msg": "NoBackgroundImage",
    "zh_msg": "无背景图像",
    "tip": "请联系售后人员",
    "ja_msg": "背景画像なし",
    "ja_tip": "アフターサービス担当までご連絡ください",
    "en_msg": "No background image",
    "en_tip": "Please contact the after-sales staff",
    "de_msg": "Kein Hintergrundbild",
    "de_tip": "Bitte wenden Sie sich an das Kundendienstpersonal",
}
ALL_ERROR["20022"] = {
    "error_code": "20022",
    "error_msg": "InvalidPrimitiveId",
    "zh_msg": "无效的Primitive id",
    "tip": "请联系售后人员",
    "ja_msg": "無効なPrimitive id",
    "ja_tip": "アフターサービス担当までご連絡ください",
    "en_msg": "Invalid primitive Id",
    "en_tip": "Please contact the after-sales staff",
    "de_msg": "Ungültige Primitiv-ID",
    "de_tip": "Bitte wenden Sie sich an das Kundendienstpersonal",
}
ALL_ERROR["20023"] = {
    "error_code": "20023",
    "error_msg": "NoLoggerFound",
    "zh_msg": "xyz logger未发现",
    "tip": "请联系售后人员",
    "ja_msg": "xyz logger不明",
    "ja_tip": "アフターサービス担当までご連絡ください",
    "en_msg": "Not found the Logger",
    "en_tip": "Please contact the after-sales staff",
    "de_msg": "Der Logger xyz wird nicht gefunden",
    "de_tip": "Bitte wenden Sie sich an das Kundendienstpersonal",
}
ALL_ERROR["20030"] = {
    "error_code": "20030",
    "error_msg": "LearningNodeError",
    "zh_msg": "learning服务异常",
    "tip": "请联系售后人员",
    "ja_msg": "learningエラー",
    "ja_tip": "アフターサービス担当までご連絡ください",
    "en_msg": "Learning service exception",
    "en_tip": "Please contact the after-sales staff",
    "de_msg": "Ausnahme der Lerndienst",
    "de_tip": "Bitte wenden Sie sich an das Kundendienstpersonal",
}
ALL_ERROR["20100"] = {
    "error_code": "20100",
    "error_msg": "Timeout",
    "zh_msg": "视觉超时",
    "tip": "请联系售后人员",
    "ja_msg": "タイムアウト",
    "ja_tip": "アフターサービス担当までご連絡ください",
    "en_msg": "Visual timeout",
    "en_tip": "Please contact the after-sales staff",
    "de_msg": "Visuelle Zeitüberschreitung",
    "de_tip": "Bitte wenden Sie sich an das Kundendienstpersonal",
}
ALL_ERROR["20101"] = {
    "error_code": "20101",
    "error_msg": "InvalidArgument",
    "zh_msg": "视觉参数错误",
    "tip": "请联系售后人员",
    "ja_msg": "ビジョンパラメータが正しくない",
    "ja_tip": "アフターサービス担当までご連絡ください",
    "en_msg": "Visual parameter error",
    "en_tip": "Please contact the after-sales staff",
    "de_msg": "Visueller Parameterfehler",
    "de_tip": "Bitte wenden Sie sich an das Kundendienstpersonal",
}
ALL_ERROR["20102"] = {
    "error_code": "20102",
    "error_msg": "FileNotFound",
    "zh_msg": "未发现文件",
    "tip": "请联系售后人员",
    "ja_msg": "ドキュメントが見つからない",
    "ja_tip": "アフターサービス担当までご連絡ください",
    "en_msg": "File does not exist",
    "en_tip": "Please contact the after-sales staff",
    "de_msg": "keine Dateien gefunden",
    "de_tip": "Bitte wenden Sie sich an das Kundendienstpersonal",
}
ALL_ERROR["29999"] = {
    "error_code": "29999",
    "error_msg": "Unknown",
    "zh_msg": "视觉服务未知错误",
    "tip": "请联系售后人员",
    "ja_msg": "視覚関連不明なエラー",
    "ja_tip": "アフターサービス担当までご連絡ください",
    "en_msg": "Vision service unknown error",
    "en_tip": "Please contact the after-sales staff",
    "de_msg": "Visueller Dienst unbekannter Fehler",
    "de_tip": "Bitte wenden Sie sich an das Kundendienstpersonal",
}
ALL_ERROR["40000"] = {
    "error_code": "40000",
    "error_msg": "Motion plan failed",
    "zh_msg": "运动规划失败",
    "tip": "请联系售后人员",
    "ja_msg": "モーションプランニング失敗",
    "ja_tip": "アフターサービス担当までご連絡ください",
    "en_msg": "Motion planning failed",
    "en_tip": "Please contact the after-sales staff",
    "de_msg": "Bewegungsplanung fehlgeschlagen",
    "de_tip": "Bitte wenden Sie sich an das Kundendienstpersonal",
}
ALL_ERROR["40001"] = {
    "error_code": "40001",
    "error_msg": "Motion plan timeout",
    "zh_msg": "运动规划超时",
    "tip": "请联系售后人员",
    "ja_msg": "モーションプランニングタイムアウト",
    "ja_tip": "アフターサービス担当までご連絡ください",
    "en_msg": "Motion Planning Timeout",
    "en_tip": "Please contact the after-sales staff",
    "de_msg": "Zeitüberschreitung bei der Bewegungsplanung",
    "de_tip": "Bitte wenden Sie sich an das Kundendienstpersonal",
}
ALL_ERROR["40002"] = {
    "error_code": "40002",
    "error_msg": "Target pose invalid",
    "zh_msg": "目标位置无效",
    "tip": "请联系售后人员",
    "ja_msg": "目標ポーズ無効",
    "ja_tip": "アフターサービス担当までご連絡ください",
    "en_msg": "Invalid target location",
    "en_tip": "Please contact the after-sales staff",
    "de_msg": "Ungültiger Zielort",
    "de_tip": "Bitte wenden Sie sich an das Kundendienstpersonal",
}
ALL_ERROR["40003"] = {
    "error_code": "40003",
    "error_msg": "Motion planner unitialized",
    "zh_msg": "运动规划服务未初始化",
    "tip": "请联系售后人员",
    "ja_msg": "モーションプランニングが初期化されない",
    "ja_tip": "アフターサービス担当までご連絡ください",
    "en_msg": "Motion Planning Service not initialized",
    "en_tip": "Please contact the after-sales staff",
    "de_msg": "Bewegungsplandienst nicht initialisiert",
    "de_tip": "Bitte wenden Sie sich an das Kundendienstpersonal",
}
ALL_ERROR["40004"] = {
    "error_code": "40004",
    "error_msg": "node is being repeated in execution",
    "zh_msg": "运动节点异常，节点重复使用",
    "tip": "请联系售后人员",
    "ja_msg": "モーションエラー、ノードが繰り返し実行される",
    "ja_tip": "アフターサービス担当までご連絡ください",
    "en_msg": "The motion node is abnormal, the node is reused",
    "en_tip": "Please contact the after-sales staff",
    "de_msg": "Anomalien des Bewegungsknotenpunkts, wiederholte Verwendung vom Knotenpunkt",
    "de_tip": "Bitte wenden Sie sich an das Kundendienstpersonal",
}
ALL_ERROR["40005"] = {
    "error_code": "40005",
    "error_msg": "Target node not in planner",
    "zh_msg": "运动节点异常，未添加结点",
    "tip": "请联系售后人员",
    "ja_msg": "モーションエラー、ノードがプラン内にない",
    "ja_tip": "アフターサービス担当までご連絡ください",
    "en_msg": "Motion node exception, no node added",
    "en_tip": "Please contact the after-sales staff",
    "de_msg": "Anomalien des Bewegungsknotenpunkts, kein Knoten hinzugefügt",
    "de_tip": "Bitte wenden Sie sich an das Kundendienstpersonal",
}
ALL_ERROR["50000"] = {
    "error_code": "50000",
    "error_msg": "Robot is in collision",
    "zh_msg": "机器人检测到碰撞",
    "tip": "1）机器人转手动，处理当前错误异常；2）判断是否为sku尺寸信息错误；3）若发生掉箱，请参考HMI用户手册处理；4）若处理不了，请联系售后人员",
    "ja_msg": "ロボットが衝突を検知した",
    "ja_tip": " 1）手動でエラーを処理する；2）skuサイズ情報エラーかどうかを判断する；3）落下が発生した場合、HMIユーザーマニュアルを参照し処理する；4）処理できない場合、アフターサービス担当までご連絡ください",
    "en_msg": "The robot detected a collision",
    "en_tip": "1) Turn the robot manually to handle the current error exception; 2) Determine whether the sku size information is wrong; 3) If the box is dropped, please refer to the HMI user manual for handling; 4) If it cannot be handled, please contact the after-sales personnel",
    "de_msg": "Der Roboter erkennt die Kollision",
    "de_tip": "Der Roboter sollt in den manuellen Modus schaltet werden, um die aktuelle Fehlerausnahme zu behandeln; 2) Stellen Sie fest, ob die Informationen zur Sku-Größe falsch sind; 3) Wenn die Box herunterfällt, lesen Sie bitte im HMI-Benutzerhandbuch nach; 4) Wenn das Problem nicht gelöst werden kann, wenden Sie sich bitte an das Kundendienstpersonal",
}
ALL_ERROR["50001"] = {
    "error_code": "50001",
    "error_msg": "Robot exceeds joint limit",
    "zh_msg": "机器人超过关节限制",
    "tip": "请联系售后人员",
    "ja_msg": "ロボットが関節制限を超えた ",
    "ja_tip": " アフターサービス担当までご連絡ください",
    "en_msg": "Robot exceeds joint limit",
    "en_tip": "Please contact the after-sales staff",
    "de_msg": "Roboter überschreitet Gelenkgrenzwerte",
    "de_tip": "Bitte wenden Sie sich an das Kundendienstpersonal",
}
ALL_ERROR["50002"] = {
    "error_code": "50002",
    "error_msg": "Robot exceeds payload limit",
    "zh_msg": "机器人超过负载限制",
    "tip": "1）机器人转手动，处理当前错误异常；2）判断物体是否超过技术协议约定的重量；3）若处理不了，请联系售后人员",
    "ja_msg": "負荷制限を超えた",
    "ja_tip": "1）手動でエラーを処理する；2）技術契約で合意された重量を超えたかどうかを判断する；3）処理できない場合、アフターサービス担当までご連絡ください",
    "en_msg": "Robot exceeds payload limit",
    "en_tip": "1) Turn the robot manually to handle the current error exception; 2) Determine whether the object exceeds the weight agreed in the technical agreement; 3) If it cannot be handled, please contact the after-sales personnel",
    "de_msg": "Roboter überschreitet Belastungsgrenzwerte",
    "de_tip": "Der Roboter sollt in den manuellen Modus schaltet werden, um die aktuelle Fehlerausnahme zu behandeln; 2) Beurteilen Sie, ob das Objekt das in der technischen Vereinbarung vereinbarte Gewicht überschreitet; 3) Wenn das Problem nicht gelöst werden kann, wenden Sie sich bitte an das Kundendienstpersonal",
}
ALL_ERROR["50003"] = {
    "error_code": "50003",
    "error_msg": "Estop from external tripper",
    "zh_msg": "机器人发生急停",
    "tip": "1）異常処理後、ロボットの緊急停止を解除する。 1）请先处理完异常后，解除机器人急停，解除方法是先将红色急停按钮旋转弹起，再按下电控柜上的复位按钮；2）重启机器人节点，再次连接；3）请参考HMI用户手册，根据现场实际情况，选择继续当前任务或取消任务，转人工处理",
    "ja_msg": "ロボットが緊急停止状態です",
    "ja_tip": "現場の故障を解消し、非常停止を解除した後、アームロボットがをリセットしてください！",
    "en_msg": "The robot has an emergency stop",
    "en_tip": "1) After the abnormality is handled, the emergency stop of the robot is canceled. 1) After handling the abnormality first, release the emergency stop of the robot. The method of release is to rotate the red emergency stop button and then press the reset button on the electric control cabinet; 2) Restart the robot node and connect again; 3) Please refer to HMI user manual, according to the actual situation on site, choose to continue the current task or cancel the task, and transfer to manual processing",
    "de_msg": "Der Roboter hat einen Notstopp",
    "de_tip": "Nachdem Sie die Anomalie behoben haben, entfernen Sie bitte die Not-Aus-Vorrichtung des Roboters. Um den roten Not-Aus-Knopf zu entriegeln, drehen Sie ihn nach oben und drücken Sie dann den Reset-Knopf am elektrischen Schaltschrank. 2) Starten Sie den Roboterknoten neu und schließen Sie ihn wieder an; 3) Lesen Sie das HMI-Benutzerhandbuch und wählen Sie je nach der tatsächlichen Situation vor Ort, ob Sie die aktuelle Aufgabe fortsetzen oder die Aufgabe abbrechen und zur manuellen Bearbeitung wechseln möchten",
}
ALL_ERROR["50004"] = {
    "error_code": "50004",
    "error_msg": "The safety gate is open now! Please close the gate before starting!",
    "zh_msg": "安全门被打开，请在开始运行前先关闭安全门！",
    "tip": "1）请先处理完异常后，关闭安全门；2）重启机器人节点，再次连接；3）请参考HMI用户手册，根据现场实际情况，选择继续当前任务或取消任务，转人工处理",
    "ja_msg": "安全ドアが開いているので、操作を始める前に閉めてください！",
    "ja_tip": "1）異常処理を先に済ませてから、安全扉を閉めてください；2）ロボットノードを再起動し、再度接続する；3）HMIのユーザーマニュアルを参照し、現場の実情に応じて、現在のタスクを継続するか、キャンセルするかを選択してください。手動に切り替えること",
    "en_msg": "The safety door is opened, please close the safety door before starting operation!",
    "en_tip": "1) Please close the safety door after handling the abnormality first; 2) Restart the robot node and connect again; 3) Please refer to the HMI user manual, and choose to continue the current task or cancel the task according to the actual situation on site, and transfer to manual processing",
    "de_msg": "Die Schutztür wurde geöffnet, bitte schließen Sie sie, bevor Sie den Betrieb starten!",
    "de_tip": "Schließen Sie die Schutztür, nachdem Sie die Störung behoben haben; 2) Starten Sie den Roboterknoten neu und stellen Sie die Verbindung wieder her; 3) Lesen Sie im HMI-Benutzerhandbuch nach und wählen Sie, ob Sie die aktuelle Aufgabe fortsetzen oder abbrechen und zur manuellen Bearbeitung wechseln möchten, je nach der tatsächlichen Situation vor Ort",
}
ALL_ERROR["50005"] = {
    "error_code": "50005",
    "error_msg": "Robot is not in home pose",
    "zh_msg": "机器人不在初始点附近，请手动将机器人移动至初始点附近",
    "tip": "1）机器人切手动，将机器人移动到初始点附近，然后再切回自动；2）重启机器人节点，再次连接；3）请参考HMI用户手册，根据现场实际情况，继续当前任务",
    "ja_msg": "ロボットが初期位置にいない、手動で初期位置に移動してください",
    "ja_tip": "1）ロボットが手動に切り替わり、初期地点まで移動した後、再び自動に切り替わる；2）ロボットノードを再起動し、再度接続する；3）HMIのユーザーマニュアルを参照し、現場の実情に応じて、現在のタスクを継続する",
    "en_msg": "The robot is not near the initial point, please manually move the robot near the initial point",
    "en_tip": "1) Switch the robot to manual, move the robot near the initial point, and then switch back to automatic; 2) Restart the robot node and connect again; 3) Please refer to the HMI user manual and continue the current task according to the actual situation",
    "de_msg": "Der Roboter befindet sich nicht in der Nähe des Startpunktes, bitte bewegen Sie den Roboter manuell zum Startpunkt",
    "de_tip": "Der Roboter sollt in den manuellen Modus schaltet werden, bewegt sich in die Nähe des Anfangspunkts und wechselt dann zurück in den automatischen Modus; 2) Starten Sie den Roboterknoten neu und stellen Sie die Verbindung wieder her; 3) Lesen Sie im HMI-Benutzerhandbuch nach und setzen Sie die aktuelle Aufgabe entsprechend der tatsächlichen Situation vor Ort fort",
}
ALL_ERROR["50006"] = {
    "error_code": "50006",
    "error_msg": "Robot failed to return to the Home point, need to hand robot near the Home point",
    "zh_msg": "机器人回Home点失败，需要手摇机器人在Home点附近",
    "tip": "1）机器人切手动，将机器人移动到初始点附近，然后再切回自动；2）重启机器人节点，再次连接；3）请参考HMI用户手册，根据现场实际情况，继续当前任务",
    "ja_msg": "ロボットが原点復帰に失敗したため、原点付近でロボットを手回しする必要がある ",
    "ja_tip": "1）ロボットが手動に切り替わり、初期地点まで移動した後、再び自動に切り替わる；2）ロボットノードを再起動し、再度接続する；3）HMIのユーザーマニュアルを参照し、現場の実情に応じて、現在のタスクを継続する",
    "en_msg": "The robot fails to return to the home point, and the hand-cranked robot is required to be near the home point",
    "en_tip": "1) Switch the robot to manual, move the robot near the initial point, and then switch back to automatic; 2) Restart the robot node and connect again; 3) Please refer to the HMI user manual and continue the current task according to the actual situation",
    "de_msg": "Der Roboter kann nicht zum Ausgangspunkt zurückkehren, daher muss der Roboter manuell in die Nähe des Ausgangspunkts bewegt werden",
    "de_tip": "Der Roboter sollt in den manuellen Modus schaltet werden, bewegt sich in die Nähe des Anfangspunkts und wechselt dann zurück in den automatischen Modus; 2) Starten Sie den Roboterknoten neu und stellen Sie die Verbindung wieder her; 3) Lesen Sie im HMI-Benutzerhandbuch nach und setzen Sie die aktuelle Aufgabe entsprechend der tatsächlichen Situation vor Ort fort",
}
ALL_ERROR["50007"] = {
    "error_code": "50007",
    "error_msg": "Robot move to capture pose fail.",
    "zh_msg": "机器人移动到拍照点失败",
    "tip": "请联系售后人员",
    "ja_msg": "ロボットが撮影点への移動に失敗した",
    "ja_tip": "アフターサービス担当までご連絡ください",
    "en_msg": "The robot failed to move to the camera point",
    "en_tip": "Please contact the after-sales staff",
    "de_msg": "Der Roboter kann nicht zum Aufnahmepunkt fahren",
    "de_tip": "Bitte wenden Sie sich an das Kundendienstpersonal",
}
ALL_ERROR["50008"] = {
    "error_code": "50008",
    "error_msg": "Robot node is error.",
    "zh_msg": "机器人节点异常",
    "tip": "1）查看机器人是否断连；2）查看机器人节点是否打开；3）重启机器人节点解除异常",
    "ja_msg": "ロボット ノードの例外",
    "ja_tip": "1) ロボットが切断されているかどうかを確認します; 2) ロボット ノードが開いているかどうかを確認します; 3) ロボット ノードを再起動して例外を解決します",
    "en_msg": "Robot node is error",
    "en_tip": "1) Check if the robot is disconnected; 2) Check if the robot node is open; 3) Restart the robot node to resolve the exception",
    "de_msg": "Bitte überprüfen Sie die Probleme von der Ausführungsphase des Roboters",
    "de_tip": "1) Prüfen, ob der Roboter getrennt ist; 2) Prüfen, ob der Roboterknoten offen ist; 3) Neustart des Roboterknotens, um die Anomalie zu beheben",
}
ALL_ERROR["60001"] = {
    "error_code": "60001",
    "error_msg": "No box pose can be returned. Please check the pallet dimensions and the box dimensions.",
    "zh_msg": "自动生成垛型规划失败，垛型为空",
    "tip": "自动生成垛型规划失败，垛型为空，请检查输入的托盘及箱子尺寸，如未解决，请联系售后人员",
    "ja_msg": "自動積み方プランニングが失敗した、積み方が空である",
    "ja_tip": "自動積み方プランニングが失敗した、積み方が空である。入力したトレイとケースのサイズを確認してください、解決できない場合、アフターサービス担当までご連絡ください",
    "en_msg": "The automatic generation of the pallet pattern planning failed, and the pallet pattern is empty",
    "en_tip": "The automatic generation of stacking type planning fails, the stacking type is empty, please check the input pallet and box size, if not resolved, please contact the after-sales personnel",
    "de_msg": "der Stapelmusterplan kann nicht automatisch erstellt werden, das Stapelmuster ist leer",
    "de_tip": "Es kann nicht automatisch ein Stapelmusterplan erstellt werden. Stapelmuster ist leer. Bitte überprüfen Sie die eingegebenen Paletten- und Kartonmaße. Falls nicht, wenden Sie sich bitte an das Kundendienstpersonal.",
}
ALL_ERROR["60002"] = {
    "error_code": "60002",
    "error_msg": "Layout stability check failed",
    "zh_msg": "自动生成垛型规划失败，稳定性检查未通过",
    "tip": "自动生成垛型规划失败，稳定性检查未通过，请检查输入的托盘及箱子尺寸，如未解决，请联系售后人员",
    "ja_msg": "自動積み方プランニングが失敗した、安定性チェックに失敗した",
    "ja_tip": "自動積み方プランニングが失敗した、安定性チェックに失敗した。入力したトレイとケースのサイズを確認してください、解決できない場合、アフターサービス担当までご連絡ください",
    "en_msg": "The automatic generation of the stacking plan failed and the stability check failed",
    "en_tip": "The automatic generation of the pallet type planning fails, and the stability check fails. Please check the input pallet and box size. If it is not resolved, please contact the after-sales personnel",
    "de_msg": "Der Stapelmusterplan kann nicht automatisch erstellt werden, die Stabilitätsprüfung ist fehlgeschlagen.",
    "de_tip": "der Stapelmusterplan konnte nicht automatisch erstellt werden. Stabilitätsprüfung fehlgeschlagen. Bitte überprüfen Sie die eingegebenen Paletten- und Kartonmaße. Falls nicht, wenden Sie sich bitte an das Kundendienstpersonal.",
}
ALL_ERROR["60003"] = {
    "error_code": "60003",
    "error_msg": "Place pose could cause part of some box out of the workspace",
    "zh_msg": "机器人放置物品的姿态超出工作空间",
    "tip": "请联系售后人员",
    "ja_msg": "ロボットの物を置くポーズがワークスペースを超えた",
    "ja_tip": "アフターサービス担当までご連絡ください",
    "en_msg": "The attitude of the robot to place the item is out of the working space",
    "en_tip": "Please check the input pallet and box size. If it is not resolved, please contact the after-sales personnel",
    "de_msg": "Die Position des Roboters zum Platzieren des Objekts überschreitet den Arbeitsbereich",
    "de_tip": "Bitte wenden Sie sich an das Kundendienstpersonal",
}
ALL_ERROR["60004"] = {
    "error_code": "60004",
    "error_msg": "Invalid config",
    "zh_msg": "配置文件无效",
    "tip": "请联系售后人员",
    "ja_msg": "無効なコンフィギュレーションファイル",
    "ja_tip": "アフターサービス担当までご連絡ください",
    "en_msg": "Configuration file is invalid",
    "en_tip": "Please check the input pallet and box size. If it is not resolved, please contact the after-sales personnel",
    "de_msg": "Ungültige Konfigurationsdatei",
    "de_tip": "Bitte wenden Sie sich an das Kundendienstpersonal",
}
ALL_ERROR["60005"] = {
    "error_code": "60005",
    "error_msg": "Data log does not exist",
    "zh_msg": "日志数据不存在",
    "tip": "请联系售后人员",
    "ja_msg": "ログデータが存在しない",
    "ja_tip": "アフターサービス担当までご連絡ください",
    "en_msg": "Log data does not exist",
    "en_tip": "Please check the input pallet and box size. If it is not resolved, please contact the after-sales personnel",
    "de_msg": "Logdaten sind nicht vorhanden",
    "de_tip": "Bitte wenden Sie sich an das Kundendienstpersonal",
}
ALL_ERROR["60006"] = {
    "error_code": "60006",
    "error_msg": "Data log already exists",
    "zh_msg": "日志数据已存在",
    "tip": "请联系售后人员",
    "ja_msg": "ログデータが既に存在する",
    "ja_tip": "アフターサービス担当までご連絡ください",
    "en_msg": "Log data already exists",
    "en_tip": "Please check the input pallet and box size. If it is not resolved, please contact the after-sales personnel",
    "de_msg": "Protokolldaten sind bereits vorhanden",
    "de_tip": "Bitte wenden Sie sich an das Kundendienstpersonal",
}
ALL_ERROR["60007"] = {
    "error_code": "60007",
    "error_msg": "Pallet Design from HMI is empty",
    "zh_msg": "从HMI上获取的该sku的垛型规划为空",
    "tip": "1）请确认HMI垛型规划是否添加有该sku信息的垛型规划；2）若未添加，请先在HMI上添加该sku垛型规划；3）请排查设置的托盘ID是否正确；4）若未解决，请联系售后人员",
    "ja_msg": "HMIから取得したskuの積み方プランが空である",
    "ja_tip": "1）HMI積み方プランにこのsku情報が追加されているかどうかご確認ください。；2）追加されていない場合は、まずHMIにsku",
    "en_msg": "The stacking plan of the sku obtained from the HMI is empty",
    "en_tip": "1) Please confirm whether the stacking plan with the sku information has been added to the HMI stacking plan; 2) If not, please add the sku stacking plan on the HMI first; 3) Please check whether the set tray ID is correct; 4) If not resolved, please contact the after-sales staff",
    "de_msg": "Der vom HMI erhaltene Sku-Stapelmusterplan ist leer",
    "de_tip": "Überprüfen Sie, ob der HMI-Stapelmusterplan die Stapelmusterplanung der Sku-Informationen hinzufügt; 2) Wenn nicht, fügen Sie bitte zuerst den Sku-Stapelmusterplan auf der HMI hinzu; 3) Überprüfen Sie, ob die Tray-ID korrekt eingestellt ist; 4) Wenn nicht, wenden Sie sich bitte an den Kundendienst",
}
ALL_ERROR["60008"] = {
    "error_code": "60008",
    "error_msg": "Waiting for the second photoshoot to end parallel event timeout",
    "zh_msg": "等待二次拍照结束并行事件超时",
    "tip": "请检查机器人执行阶段的问题",
    "ja_msg": "2 番目の写真撮影が並行イベントのタイムアウトを終了するのを待っています",
    "ja_tip": "ボット実行フェーズの問題を確認してください",
    "en_msg": "Waiting for the second photoshoot to end parallel event timeout",
    "en_tip": "Please check for issues in the bot execution phase",
    "de_msg": "Zeitüberschreitung beim Warten auf das Ende des Sekundärfoto-Parallelereignisses",
    "de_tip": "Bitte überprüfen Sie die Probleme von der Ausführungsphase des Roboters",
}
ALL_ERROR["60009"] = {
    "error_code": "60009",
    "error_msg": "Timed out waiting for the robot to reach the grab point parallel event",
    "zh_msg": "等待机器人到达抓取点并行事件超时",
    "tip": "请检查机器人执行阶段的问题",
    "ja_msg": "ロボットがグラブ ポイントに到達するのを待ってタイムアウトしました並列イベント",
    "ja_tip": "ボット実行フェーズの問題を確認してください",
    "en_msg": "Timed out waiting for the robot to reach the grab point parallel event",
    "en_tip": "Please check for issues in the bot execution phase",
    "de_msg": "Zeitüberschreitung beim  Warten auf das Erreichen des Greifpunkts  von Robter Parallel-Event",
    "de_tip": "Bitte überprüfen Sie die Probleme von der Ausführungsphase des Roboters",
}
ALL_ERROR["60010"] = {
    "error_code": "60010",
    "error_msg": "generate_grasp_plan_new has no grasp_plans or object_poses",
    "zh_msg": "纸箱抓取规划计算抓取和放置方式失败",
    "tip": "没有找到抓取规划和放置位置的对应关系. 1）请检查下发的物体重量是否过大；2）请检查视觉识别的物体之间是否有重叠交叉，影响抓取",
    "ja_msg": "カートン ピック プランでピック アンド プレース モードの計算に失敗しました",
    "ja_tip": "把持計画と配置位置の対応関係が見当たらなかった 1) 納入物の重量が大きすぎないか確認してください 2) 視認物同士が重なり合っていないか確認してください 把持に影響します",
    "en_msg": "generate_grasp_plan_new has no grasp_plans or object_poses",
    "en_tip": "The corresponding relationship between the grasping plan and the placement position was not found. 1) Please check whether the weight of the delivered object is too large; 2) Please check whether there is any overlap between the visually recognized objects, which affects the grasping",
    "de_msg": "Planungsberechnung für das Greifen von Kartons bei Greif- und Ablagefehlern",
    "de_tip": "1) Prüfen Sie, ob das Gewicht des Objekts zu groß ist; 2) Prüfen Sie, ob es Überschneidungen und Kreuzungen zwischen den visuell erkannten Objekten gibt, die das Greifen beeinträchtigen",
}
ALL_ERROR["60011"] = {
    "error_code": "60011",
    "error_msg": "Timeout waiting for the robot to leave the camera's field of view",
    "zh_msg": "等待机器人离开相机视野超时",
    "tip": "请检查机器人执行阶段的问题",
    "ja_msg": "ロボットがカメラの視野から出るのを待つタイムアウト",
    "ja_tip": "ボット実行フェーズの問題を確認してください",
    "en_msg": "Timeout waiting for the robot to leave the camera's field of view",
    "en_tip": "Please check for issues in the bot execution phase",
    "de_msg": "Zeitüberschreitung beim Warten darauf, dass der Roboter das Sichtfeld der Kamera verlässt",
    "de_tip": "Bitte überprüfen Sie die Probleme von der Ausführungsphase des Roboters",
}
ALL_ERROR["69998"] = {
    "error_code": "69998",
    "error_msg": "Parameter setting error",
    "zh_msg": "参数设置错误",
    "tip": "请联系售后人员",
    "ja_msg": "パラメータ設定エラー",
    "ja_tip": "アフターサービス担当までご連絡ください",
    "en_msg": "Parameter setting error",
    "en_tip": "Please check the input pallet and box size. If it is not resolved, please contact the after-sales personnel",
    "de_msg": "Fehler bei der Parametereinstellung",
    "de_tip": "Bitte wenden Sie sich an das Kundendienstpersonal",
}
ALL_ERROR["69999"] = {
    "error_code": "69999",
    "error_msg": "Program inner error",
    "zh_msg": "程序内部错误",
    "tip": "请联系售后人员",
    "ja_msg": "プログラム内部エラー",
    "ja_tip": "アフターサービス担当までご連絡ください",
    "en_msg": "Internal error",
    "en_tip": "Please check the input pallet and box size. If it is not resolved, please contact the after-sales personnel",
    "de_msg": "Interne Programmfehler",
    "de_tip": "Bitte wenden Sie sich an das Kundendienstpersonal",
}
ALL_ERROR["70000"] = {
    "error_code": "70000",
    "error_msg": "One or more barcodes can’t be correctly located",
    "zh_msg": "条码通过视觉系统定位异常",
    "tip": "人工通过HMI上的local helper功能进行辅助标注",
    "ja_msg": "ビジョンシステムでバーコードのポジション異常",
    "ja_tip": "HMIのローカルヘルパー機能による手動ラベリング",
    "en_msg": "The barcode locates abnormally through the vision system",
    "en_tip": "Manually perform auxiliary labeling through the local helper function on the HMI",
    "de_msg": "Der Barcode hat Anomalien bei der Lokalisierung durch das visuelle System",
    "de_tip": "Das manuelle Kennzeichen durch lokale Hilfsfunktion auf der HMI",
}
ALL_ERROR["70001"] = {
    "error_code": "70001",
    "error_msg": "The barcode can’t be correctly read",
    "zh_msg": "无法读取条码，可能因为条码破损或者打印质量偏低",
    "tip": "请参考HMI用户手册，根据现场实际情况，选择继续当前任务或取消任务，转人工处理",
    "ja_msg": "バーコードが読み取れない。バーコードが壊れているか、印刷品質が低い可能性がある。",
    "ja_tip": "HMIのユーザーマニュアルを参照し、現場の状況に応じて、現在のタスクを継続するか、キャンセルしてマニュアル処理に切り替えるかを選択してください",
    "en_msg": "The barcode cannot be read, possibly because the barcode is damaged or the print quality is low",
    "en_tip": "Please refer to the HMI user manual, and choose to continue the current task or cancel the task according to the actual situation on site, and transfer to manual processing",
    "de_msg": "Der Barcode kann nicht gelesen werden, möglicherweise weil der Barcode beschädigt ist oder die Druckqualität schlecht ist.",
    "de_tip": "Bitte lesen Sie das HMI-Benutzerhandbuch und wählen Sie je nach der tatsächlichen Situation vor Ort, ob Sie die aktuelle Aufgabe fortsetzen oder abbrechen und zur manuellen Bearbeitung wechseln möchten",
}
ALL_ERROR["70002"] = {
    "error_code": "70002",
    "error_msg": "The tote tool goes wrong",
    "zh_msg": "夹具异常：检测或控制失败",
    "tip": "1）请排查夹具检测或控制异常；2）请参考HMI用户手册，根据现场实际情况，选择继续当前任务或取消任务，转人工处理",
    "ja_msg": "ハンドの異常：検出または制御の失敗",
    "ja_tip": "1）ハンドの検出や制御の異常がないかをご確認ください。；2）HMIのユーザーマニュアルを参照し、現場の状況に応じて、現在のタスクを継続するか、キャンセルしてマニュアル処理に切り替えるかを選択してください",
    "en_msg": "Fixture exception: detection or control failure",
    "en_tip": "1) Please check the fixture detection or control abnormality; 2) Please refer to the HMI user manual, according to the actual situation on site, choose to continue the current task or cancel the task, and transfer to manual processing",
    "de_msg": "Anomalie in der Vorrichtung: Erkennungs- oder Steuerungsfehler",
    "de_tip": "Überprüfen Sie, ob die Erkennung oder Steuerung der Vorrichtung nicht normal ist; 2) Lesen Sie das HMI-Benutzerhandbuch und wählen Sie je nach der tatsächlichen Situation vor Ort, ob Sie die aktuelle Aufgabe fortsetzen oder abbrechen und zur manuellen Bearbeitung wechseln möchten",
}
ALL_ERROR["70003"] = {
    "error_code": "70003",
    "error_msg": "Object is dropped",
    "zh_msg": "检测到发生掉箱，物体掉落",
    "tip": "1）请参考HMI用户手册处理掉箱异常及恢复；2）请根据现场实际情况，机器人拍急停，处理掉箱异常，注意安全操作规范；3）若未解决，请联系售后人员",
    "ja_msg": "箱・物体落下が検出された",
    "ja_tip": "1）HMIのユーザーマニュアルを参照し、処理してください。；2）現場の状況に応じてロボットを緊急停止して、箱落下の異常に対処し、安全操作の規範に注意してください。；3）解決されない場合は、アフターサービス担当までご連絡ください",
    "en_msg": "A box drop is detected and the object is dropped",
    "en_tip": ") Please refer to the HMI user manual to deal with the abnormal box drop and recovery; 2) Please according to the actual situation on site, the robot shoots an emergency stop, handle the box drop exception, and pay attention to the safety operation specifications; 3) If it is not resolved, please contact the after-sales personnel",
    "de_msg": "Es wird detektiert, dass der Karton fällt, das Objekt herunterfällt.",
    "de_tip": "Beziehen Sie sich bitte auf das HMI-Benutzerhandbuch, um Ausnahmen und die Wiederherstellung des Falles zu handhaben; 2) Entsprechend der tatsächlichen Situation vor Ort muss der Roboter in einem Notfall anhalten, den anormalen Fall eines Sturzes handhaben und die Spezifikationen für den Sicherheitsbetrieb beachten; 3) Falls nicht, wenden Sie sich bitte an das Kundendienstpersonal",
}
ALL_ERROR["70004"] = {
    "error_code": "70004",
    "error_msg": "The tool switch goes wrong",
    "zh_msg": "快换信号异常：检测或控制失败",
    "tip": "1）请排查快换信号检测或控制异常；2）请参考HMI用户手册，根据现场实际情况，选择继续当前任务或取消任务，转人工处理",
    "ja_msg": "クイックチェンジ信号の異常：検出または制御の故障",
    "ja_tip": "1）クイックチェンジ信号の検出や制御に異常がないかを確認してください；2）HMIのユーザーマニュアルを参照し、現場の状況に応じて、現在のタスクを継続するか、キャンセルしてマニュアル処理に切り替えるかを選択してください",
    "en_msg": "Abnormal quick change signal: detection or control failure",
    "en_tip": "1) Please check the quick-change signal detection or control abnormality; 2) Please refer to the HMI user manual, according to the actual situation on site, choose to continue the current task or cancel the task, and transfer to manual processing",
    "de_msg": "Abnormales Schnellwechselsignal: Erkennungs- oder Steuerungsfehler",
    "de_tip": "Überprüfen Sie, ob die Erkennung oder Steuerung des Schnellwechselsignals anormal ist; 2) Schlagen Sie im HMI-Benutzerhandbuch nach und wählen Sie je nach der tatsächlichen Situation vor Ort, ob Sie die aktuelle Aufgabe fortsetzen oder abbrechen und zur manuellen Bearbeitung wechseln möchten",
}
ALL_ERROR["70006"] = {
    "error_code": "70006",
    "error_msg": "Failed to pick the item",
    "zh_msg": "抓取失败",
    "tip": "1）请排查夹具检测或控制异常；2）请参考HMI用户手册，根据现场实际情况，选择继续当前任务或取消任务，转人工处理",
    "ja_msg": "ピックアップ失敗",
    "ja_tip": "1）ハンドの検出や制御の異常がないかをご確認ください。；2）HMIのユーザーマニュアルを参照し、現場の状況に応じて、現在のタスクを継続するか、キャンセルしてマニュアル処理に切り替えるかを選択してください",
    "en_msg": "Failed to pick the item",
    "en_tip": "1) Please check the fixture detection or control abnormality; 2) Please refer to the HMI user manual, according to the actual situation on site, choose to continue the current task or cancel the task, and transfer to manual ",
    "de_msg": "Greifen fehlgeschlagen",
    "de_tip": "Überprüfen Sie, ob die Erkennung oder Steuerung der Vorrichtung anormal ist; 2) Lesen Sie das HMI-Benutzerhandbuch und wählen Sie je nach der tatsächlichen Situation vor Ort, ob Sie die aktuelle Aufgabe fortsetzen oder abbrechen und zur manuellen Bearbeitung wechseln möchten",
}
ALL_ERROR["70007"] = {
    "error_code": "70007",
    "error_msg": "Two tools are detected not on the rack",
    "zh_msg": "无法检测两个工具在快换托上",
    "tip": "1）请排查快换托上工具检测是否异常；2）请参考HMI用户手册，根据现场实际情况，选择继续当前任务或取消任务，转人工处理",
    "ja_msg": "クイックチェンジトレイ上の2つの工具を検出できない",
    "ja_tip": "1）クイックチェンジ・トレーのツール検出に異常がないかを確認してください；2）HMIのユーザーマニュアルを参照し、現場の状況に応じて、現在のタスクを継続するか、キャンセルしてマニュアル処理に切り替えるかを選択してください",
    "en_msg": "Unable to detect two tools on quick change tray",
    "en_tip": "1) Please check whether the detection of the quick change carrier tool is abnormal; 2) Please refer to the HMI user manual, and choose to continue the current task or cancel the task according to the actual situation on site, and transfer to manual processing",
    "de_msg": "Zwei Werkzeuge können auf der Schnellwechselhalterung nicht erkannt werden",
    "de_tip": "Überprüfen Sie, ob die Werkzeuge an der Schnellwechselhalterung anormal sind; 2) Lesen Sie im HMI-Benutzerhandbuch nach und wählen Sie, ob Sie die aktuelle Aufgabe fortsetzen oder abbrechen und zur manuellen Bearbeitung wechseln wollen, je nach der tatsächlichen Situation vor Ort",
}
ALL_ERROR["70008"] = {
    "error_code": "70008",
    "error_msg": "No tool on the robot, make sure at least one tool is attached",
    "zh_msg": "检测到机器人无末端执行器",
    "tip": "1）请排查机器人末端执行器检测是否异常；2）请参考HMI用户手册，根据现场实际情况，选择继续当前任务或取消任务，转人工处理",
    "ja_msg": "ロボットにエンドツールが検出されない",
    "ja_tip": "1）ロボットのエンドツール検出に異常がないかを確認してください。；2）HMIのユーザーマニュアルを参照し、現場の状況に応じて、現在のタスクを継続するか、キャンセルしてマニュアル処理に切り替えるかを選択してください",
    "en_msg": "Robot has no end effector detected",
    "en_tip": "1) Please check whether the robot end-effector detection is abnormal; 2) Please refer to the HMI user manual, according to the actual situation on site, choose to continue the current task or cancel the task, and transfer to manual processing",
    "de_msg": "Es wird festgestellt, dass der Roboter keinen Endeffektor hat",
    "de_tip": "Überprüfen Sie, ob die Erkennung des Roboter-Endeffktors anormal ist; 2) Schlagen Sie im HMI-Benutzerhandbuch nach und wählen Sie, ob Sie die aktuelle Aufgabe fortsetzen oder abbrechen und zur manuellen Bearbeitung wechseln wollen, je nach den tatsächlichen Gegebenheiten vor Ort",
}
ALL_ERROR["70009"] = {
    "error_code": "70009",
    "error_msg": "Failed to release the carton",
    "zh_msg": "无法释放纸箱",
    "tip": "1）请排查吸盘工具是否异常；2）请参考HMI用户手册，根据现场实际情况，选择继续当前任务或取消任务，转人工处理",
    "ja_msg": "段ボール箱の放置ができない",
    "ja_tip": "1）吸盤ツールに異常がないかを確認してください；2）HMIのユーザーマニュアルを参照し、現場の状況に応じて、現在のタスクを継続するか、キャンセルしてマニュアル処理に切り替えるかを選択してください",
    "en_msg": "Unable to release carton",
    "en_tip": "1) Please check whether the suction cup tool is abnormal; 2) Please refer to the HMI user manual, according to the actual situation on site, choose to continue the current task or cancel the task, and transfer to manual processing",
    "de_msg": "Karton kann nicht freigegeben werden",
    "de_tip": "Überprüfen Sie, ob das Saugnapfwerkzeug nicht in Ordnung ist; 2) Schlagen Sie im HMI-Benutzerhandbuch nach und wählen Sie, ob Sie die aktuelle Aufgabe fortsetzen oder abbrechen und zur manuellen Bearbeitung wechseln wollen, je nach der tatsächlichen Situation vor Ort",
}
ALL_ERROR["70010"] = {
    "error_code": "70010",
    "error_msg": "Pressure is too low",
    "zh_msg": "气压值过低",
    "tip": "请检查空压机阀门是否打开，气管是否漏气，管路是否有挤压折叠漏气，压力表是否正常",
    "ja_msg": "圧力が低すぎる",
    "ja_tip": "エアコンプレッサーのバルブが開いているか、エア配管に漏れがないか、配管が押し出され折れていないか、圧力計は正常かなどをご確認ください",
    "en_msg": "Air pressure too low",
    "en_tip": "Please check whether the air compressor valve is open, whether the air pipe is leaking, whether the pipeline is squeezed and folded, and whether the pressure gauge is normal.",
    "de_msg": "Der Luftdruckwert ist zu niedrig",
    "de_tip": "Prüfen Sie, ob das Ventil des Luftkompressors geöffnet ist, ob die Luftleitung undicht ist, ob die Leitung gequetscht und gefaltet ist und ob das Manometer normal ist.",
}
ALL_ERROR["70011"] = {
    "error_code": "70011",
    "error_msg": "Pressure is too high",
    "zh_msg": "气压值过高",
    "tip": "请检查空压机预设气压值是否过大，压力表是否正常",
    "ja_msg": "圧力が高すぎる",
    "ja_tip": "コンプレッサーの設定空気圧が高すぎないか、圧力計は正常か、ご確認ください。",
    "en_msg": "Air pressure too high",
    "en_tip": "Please check whether the preset air pressure value of the air compressor is too large, and whether the pressure gauge is normal",
    "de_msg": "Der Luftdruckwert ist zu hoch",
    "de_tip": "Prüfen Sie, ob der voreingestellte Luftdruck des Luftkompressors zu hoch ist und ob das Manometer normal ist.",
}
ALL_ERROR["70012"] = {
    "error_code": "70012",
    "error_msg": "Barcode scanner can not start",
    "zh_msg": "扫码器无法打开",
    "tip": "请重启扫码器，若未解决，请联系售后人员",
    "ja_msg": "スキャナーが開けない",
    "ja_tip": "スキャナーを再起動してください、解決されない場合は、アフターサービス担当までご連絡ください",
    "en_msg": "The scanner cannot be opened",
    "en_tip": "Please restart the scanner, if not resolved, please contact the after-sales staff",
    "de_msg": "Scanner kann nicht angeschaltet werden",
    "de_tip": "Starten Sie den Codescanner neu. Wenn das Problem nicht gelöst werden kann, wenden Sie sich bitte an das Kundendienstpersonal",
}
ALL_ERROR["70013"] = {
    "error_code": "70013",
    "error_msg": "Barcode  scanner can not stop",
    "zh_msg": "扫码器无法停止",
    "tip": "请重启扫码器，若未解决，请联系售后人员",
    "ja_msg": "スキャナーが止まらない",
    "ja_tip": "スキャナーを再起動してください、解決されない場合は、アフターサービス担当までご連絡ください",
    "en_msg": "The scanner cannot be stopped",
    "en_tip": "Please restart the scanner, if not resolved, please contact the after-sales staff",
    "de_msg": "Scanner kann nicht ausgeschaltet werden",
    "de_tip": "Starten Sie den Codescanner neu. Wenn das Problem nicht gelöst werden kann, wenden Sie sich bitte an das Kundendienstpersonal",
}
ALL_ERROR["70014"] = {
    "error_code": "70014",
    "error_msg": "More than one tool missing",
    "zh_msg": "超过一个吸盘不在快换位",
    "tip": "1）请检查快换位的吸盘以及光电传感器是否异常；2）请参考HMI用户手册，根据现场实际情况，选择继续当前任务或取消任务，转人工处理",
    "ja_msg": "複数のツールがクイックチェンジ位置にない",
    "ja_tip": "1）クイックチェンジ吸盤と光電センサーに異常がないかをご確認ください；2）HMIのユーザーマニュアルを参照し、現場の状況に応じて、現在のタスクを継続するか、キャンセルしてマニュアル処理に切り替えるかを選択してください",
    "en_msg": "More than one suction cup is not in fast transposition",
    "en_tip": "1) Please check whether the quick-change suction cup and photoelectric sensor are abnormal; 2) Please refer to the HMI user manual, and choose to continue the current task or cancel the task according to the actual situation on site, and transfer to manual processing",
    "de_msg": "Mehrere Saugnäpfe nicht in der Schnellwechselnstelle",
    "de_tip": "Überprüfen Sie, ob der Saugnapf in der Schnellwechselnstelle und der fotoelektrische Sensor abnormal sind; 2) Schlagen Sie im HMI-Benutzerhandbuch nach und wählen Sie je nach der tatsächlichen Situation vor Ort, ob Sie die aktuelle Aufgabe fortsetzen oder die Aufgabe abbrechen und zur manuellen Verarbeitung wechseln möchten",
}
ALL_ERROR["70015"] = {
    "error_code": "70015",
    "error_msg": "Weight sensor error",
    "zh_msg": "重力传感器故障",
    "tip": "1）请检查重力传感器线路；2）请参考HMI用户手册，根据现场实际情况，选择继续当前任务或取消任务，转人工处理；若未解决，请联系售后人员",
    "ja_msg": "重力センサー故障",
    "ja_tip": "1）重力センサーの配線を確認してください；2）HMIのユーザーマニュアルを参照し、現場の状況に応じて、現在のタスクを継続するか、キャンセルしてマニュアル処理に切り替えるかを選択してください；解決されない場合は、アフターサービス担当までご連絡ください",
    "en_msg": "Gravity sensor failure",
    "en_tip": "1) Please check the gravity sensor circuit; 2) Please refer to the HMI user manual, and choose to continue the current task or cancel the task according to the actual situation on site, and transfer it to manual processing; if it is not resolved, please contact the after-sales personnel",
    "de_msg": "Fehler des Schwerkraftsensors",
    "de_tip": "Bitte überprüfen Sie den Stromkreis des Schwerkraftsensors; 2) Bitte lesen Sie im HMI-Benutzerhandbuch nach und wählen Sie je nach der tatsächlichen Situation vor Ort, ob Sie die aktuelle Aufgabe fortsetzen oder abbrechen und zur manuellen Bearbeitung übergehen möchten; falls nicht, wenden Sie sich bitte an den Kundendienst",
}
ALL_ERROR["70016"] = {
    "error_code": "70016",
    "error_msg": "More than one sku dropping in different tote",
    "zh_msg": "检测到sku掉落在两个以上的料箱",
    "tip": "1）请检查是否有多个sku掉入不同的料箱中；2）请检查光栅传感器线路；3）若未解决，请联系售后人员",
    "ja_msg": "2つ以上のコンテナにドロップされたスクーの検出",
    "ja_tip": "1）複数のskuが異なるビンにドロップされていないか確認してください。；2）ラスターセンサ回路をご確認ください；3）解決されない場合は、アフターサービス担当までご連絡ください",
    "en_msg": "Detected sku dropped in more than two bins",
    "en_tip": "1) Please check if there are multiple SKUs falling into different bins; 2) Please check the grating sensor circuit; 3) If it is not resolved, please contact the after-sales personnel",
    "de_msg": "Es wird festgestellt, dass die Sku auf mehr als zwei Lagerkisten fällt",
    "de_tip": "Prüfen Sie, ob mehrere Skus in verschiedene Lagerkisten fallen; 2) Prüfen Sie die Schaltung des Gittersensors; 3) Falls nicht, wenden Sie sich bitte an das Kundendienstpersonal",
}
ALL_ERROR["70017"] = {
    "error_code": "70017",
    "error_msg": "Raster is still triggered",
    "zh_msg": "检测到光栅长触发",
    "tip": "请检查料箱中sku是否遮挡了光栅，若未解决，请联系售后人员",
    "ja_msg": "ラスターはまだトリガーされる",
    "ja_tip": "ビン内のskuがラスターをブロックしていないか確認してください、解決されない場合は、アフターサービス担当までご連絡ください",
    "en_msg": "Raster long trigger detected",
    "en_tip": "Please check whether the grating is blocked by the sku in the material box. If it is not resolved, please contact the after-sales ",
    "de_msg": "Gitter Lange-Trigger erkannt",
    "de_tip": "Bitte prüfen Sie, ob die Sku in der Lagerkisten das Gitter blockiert. Wenn nicht, wenden Sie sich bitte an das Kundendienstpersonal",
}
ALL_ERROR["70018"] = {
    "error_code": "70018",
    "error_msg": "The communication between the industrial computer and the PLC is disconnected or the address bit of the operation is wrong",
    "zh_msg": "工控机与PLC之间通信断连或操作的地址位错误",
    "tip": "请检查工控机与PLC通信是否正常，查看工控机操作的PLC地址位是否正确",
    "ja_msg": "IPCとPLC間の通信が切断された、または操作のアドレスが間違っている",
    "ja_tip": "IPCとPLCの通信が正常かどうか、IPCが操作するPLCアドレスが正しいかどうかを確認してください。",
    "en_msg": "The communication between the industrial computer and the PLC is disconnected or the address bit of the operation is wrong",
    "en_tip": "Please check whether the communication between the industrial computer and the PLC is normal, and check whether the PLC address bits operated by the industrial computer are correct.",
    "de_msg": "Die Kommunikation zwischen IPC und SPS ist unterbrochen oder Adressbits der Operation ist falsch",
    "de_tip": "Prüfen Sie, ob die Kommunikation zwischen IPC und SPS normal ist und ob das SPS-Adressbit des IPC-Betriebs korrekt ist.",
}
ALL_ERROR["70019"] = {
    "error_code": "70019",
    "error_msg": "The signal values for checking if the items hav have been dropped are inconsistent",
    "zh_msg": "掉箱检测信号不一致",
    "tip": "请检查所有掉箱检测信号是否正确",
    "ja_msg": "落下箱の検出信号が不一致である",
    "ja_tip": "検出信号がすべて正しいことを確認してください",
    "en_msg": "Inconsistent drop detection signal",
    "en_tip": "Please check that all drop detection signals are correct",
    "de_msg": "Inkonsistente Erkennungssignale für den Sturz von Kartons",
    "de_tip": "Prüfen Sie, ob das Erkennungssignal in allen Fällen korrekt ist.",
}
ALL_ERROR["70020"] = {
    "error_code": "70020",
    "error_msg": "Can not read plc signal, please check io node is alive",
    "zh_msg": "无法获取IO信号",
    "tip": "检查io节点是否处于活动状态，检查工控机与PLC之间的通信是否正常",
    "ja_msg": "IO信号の取得ができない",
    "ja_tip": "ioノードがアクティブかどうか、IPCとPLC間の通信が正常かどうか確認してください",
    "en_msg": "Can't get IO signal",
    "en_tip": "Check whether the io node is active, check whether the communication between the industrial computer and the PLC is normal",
    "de_msg": "IO-Signal kann nicht empfangen werden",
    "de_tip": "Prüfen Sie, ob der io-Knoten aktiviert ist und ob die Kommunikation zwischen IPC und SPS normal ist.",
}
ALL_ERROR["70021"] = {
    "error_code": "70021",
    "error_msg": "The number of failed scan barcodes exceeds the specified number of times",
    "zh_msg": "扫码失败次数超过规定次数",
    "tip": "放弃此任务，人工转移托盘或料箱至人工位处理",
    "ja_msg": "指定回数以上のスキャンナー失敗",
    "ja_tip": "この作業を放棄し、パレットやビンをハンドリングのために手動で移動させる",
    "en_msg": "The number of failed scan codes exceeds the specified number of times",
    "en_tip": "Abandon this task and manually transfer pallets or bins to manual processing",
    "de_msg": "Die Anzahl der Fehler beim Scannen des Codes übersteigt die angegebene Anzahl",
    "de_tip": "Brechen Sie diese Aufgabe ab und bringen Sie die Palette oder Lagerkiste manuell in die entsprechende Position zur Verarbeitung",
}
ALL_ERROR["70022"] = {
    "error_code": "70022",
    "error_msg": "Picking workspace is empty",
    "zh_msg": "检测到抓取工作空间已空",
    "tip": "请检查抓取工作空间中是否有物体",
    "ja_msg": "ピックアップのワークスペースが空であると検出される",
    "ja_tip": "ワークスペースに物があるかどうか確認してください。",
    "en_msg": "Crawl workspace detected to be empty",
    "en_tip": "Please check if there are objects in the grab workspace",
    "de_msg": "Greifen-Arbeitsbereich is leer",
    "de_tip": "Prüfen Sie, ob sich Objekte im Greifen-Arbeitsbereich befinden",
}
ALL_ERROR["70023"] = {
    "error_code": "70023",
    "error_msg": "Placing workspace is full",
    "zh_msg": "检测到放置工作空间已满",
    "tip": "请保证放置空间有足够的空间放置SKU",
    "ja_msg": "配置ワークスペースがいっぱいであること",
    "ja_tip": "SKUの十分なスペースを確保してください。",
    "en_msg": "Placement workspace detected to be full",
    "en_tip": "Please ensure that there is enough space to place the SKU",
    "de_msg": "Platzieren-Arbeitsbereich voll erkannt",
    "de_tip": "Vergewissern Sie sich, dass genügend Platz für SKU vorhanden ist.",
}
ALL_ERROR["80001"] = {
    "error_code": "80001",
    "error_msg": "Received the message of depalletizing, but detected that depalletizing is not in place",
    "zh_msg": "收到拆垛消息，但检测到拆垛位托盘未到位",
    "tip": "请检测码垛位托盘是否到位，检测PLC信号是否到位，检测工控机与PLC是否断连，检测光电传感器设备是否有故障",
    "ja_msg": "デパレタスクを受信したが、デパレ位置にパレット未到着です",
    "ja_tip": "パレットが所定の位置にあるかどうか、PLC信号が所定の位置にあるかどうか、IPCがPLCから外れていないかどうか、光電センサー装置に異常がないかどうか確認してください",
    "en_msg": "Received a depalletizing message, but detected that the depalletizing position pallet is not in place",
    "en_tip": "Please check whether the pallet at the palletizing position is in place, check whether the PLC signal is in place, check whether the industrial computer is disconnected from the PLC, and check whether the photoelectric sensor equipment is faulty",
    "de_msg": "Die De-Palettierungsaufgabe wird empfangen, aber es wird festgestellt, dass die Pallete an der Greifposition nicht einsatzbereit ist.",
    "de_tip": "Überprüfen Sie, ob die Paletten vorhanden ist, ob das PLC-Signal vorhanden ist, ob der IPC von der SPS getrennt ist und ob der optoelektronische Sensor defekt ist.",
}
ALL_ERROR["80002"] = {
    "error_code": "80002",
    "error_msg": "Received palletizing message, but detected pallets not in place",
    "zh_msg": "收到码垛消息，但检测到码垛位托盘未到位",
    "tip": "请检测码垛位托盘是否到位，检测PLC信号是否到位，检测工控机与PLC是否断连，检测光电传感器设备是否有故障",
    "ja_msg": "パレタイズタスクを受信したが、パレタイズ位置にパレット未到着です",
    "ja_tip": "パレットが所定の位置にあるかどうか、PLC信号が所定の位置にあるかどうか、IPCがPLCから外れていないかどうか、光電センサー装置に異常がないかどうか確認してください",
    "en_msg": "Palletizing message received, but pallet detected not in place",
    "en_tip": "Please check whether the pallet at the palletizing position is in place, check whether the PLC signal is in place, check whether the industrial computer is disconnected from the PLC, and check whether the photoelectric sensor equipment is faulty",
    "de_msg": "Die Palettierungsaufgabe wird empfangen, aber es wird festgestellt, dass die Palette an der Palettierposition nicht einsatzbereit ist.",
    "de_tip": "Überprüfen Sie, ob die Paletten vorhanden ist, ob das PLC-Signal vorhanden ist, ob der IPC von der SPS getrennt ist und ob der optoelektronische Sensor defekt ist.",
}
ALL_ERROR["80003"] = {
    "error_code": "80003",
    "error_msg": "The sku info given by the WCS may be wrong",
    "zh_msg": "WCS下发的纸箱尺寸与实际尺寸不符",
    "tip": "1）请检查WCS下发的信息是否正确，请检查来料是否正确；2）请根据现场实际，WCS取消订单，修改尺寸信息后重新下发；3）请参考HMI用户手册，根据现场实际情况，进行处理；4）请判断是否要进行转人工处理；5）若未解决，请联系售后人员",
    "ja_msg": "WCSが発行する箱のサイズと実際のサイズが一致しない",
    "ja_tip": "1）WCSから送信された情報が正しいかどうか、来たSKUが正しいかどうか確認してください；2）現場の実際状況に従って、WCSがオーダーをキャンセルし、サイズ情報を変更した後、再度送信してください；3）HMIのユーザーマニュアルをご参照の上、現場の実情に合わせてお取り扱いください。；4）手動処理を行うかどうか判断してください；5）解決されない場合は、アフターサービス担当までご連絡ください",
    "en_msg": "The size of the carton issued by WCS does not match the actual size",
    "en_tip": "1) Please check whether the information issued by WCS is correct, and please check whether the incoming materials are correct; 2) Please cancel the order according to the actual situation on site, modify the size information and issue it again; 3) Please refer to the HMI user manual, according to the actual situation on site , and deal with it; 4) Please judge whether to transfer to manual processing; 5) If it is not resolved, please contact the after-sales personnel",
    "de_msg": "Die vom WCS ausgegebene Kartongröße stimmt nicht mit der tatsächlichen Größe überein.",
    "de_tip": "Bitte prüfen Sie, ob die von WCS herausgegebenen Informationen korrekt sind und ob die eingehenden Materialien richtig sind; 2) Entsprechend der tatsächlichen Situation vor Ort storniert WCS den Auftrag, ändert die Größenangaben und gibt ihn erneut heraus; 3) Bitte lesen Sie das HMI-Benutzerhandbuch und handhaben Sie es entsprechend der tatsächlichen Situation vor Ort; 4) Bitte beurteilen Sie, ob Sie auf manuelle Bearbeitung umstellen müssen; 5) Wenn nicht, wenden Sie sich bitte an den Kundendienst",
}
ALL_ERROR["80004"] = {
    "error_code": "80004",
    "error_msg": "Invalid task type",
    "zh_msg": "任务错误：WCS下发无效任务类型，任务无法进行",
    "tip": "WCS下发的任务订单错误，请发送正确的任务订单",
    "ja_msg": "タスクエラー：WCSによって無効なタスクタイプが発行された。タスクを実行できません",
    "ja_tip": "お客様オーダーエラー、正確なオーダーを送信してください",
    "en_msg": "Task error: Invalid task type issued by WCS, the task cannot be performed",
    "en_tip": "The task order issued by WCS is wrong, please send the correct task order",
    "de_msg": "Fehler bei der Aufgabe: Der vom WCS gesendete Auftragstyp ist ungültig und kann nicht ausgeführt werden.",
    "de_tip": "Der von WCS aufgegebene Arbeitsauftrag ist falsch. Bitte senden Sie den richtigen Auftrag",
}
ALL_ERROR["80005"] = {
    "error_code": "80005",
    "error_msg": "Invalid pick workspace_id",
    "zh_msg": "任务错误：WCS下发抓取工作空间不存在，任务无法进行",
    "tip": "WCS下发的任务订单错误，请发送正确的任务订单",
    "ja_msg": "タスクエラー：WCSによってつかみ取り空間がないタスクが送信され、タスクを実行できません",
    "ja_tip": "WCSが送信したタスクオーダーが誤っているため、正しいタスクオーダーを送信してください",
    "en_msg": "Task error: The crawl workspace issued by WCS does not exist, and the task cannot be performed",
    "en_tip": "The task order issued by WCS is wrong, please send the correct task order",
    "de_msg": "Fehler bei der Aufgabe: Der verteilte Erfassungsarbeitsbereich des WCS ist nicht vorhanden, die Aufgabe kann nicht ausgeführt werden",
    "de_tip": "Der von WCS herausgegebene Arbeitsauftrag ist falsch. Bitte senden Sie den richtigen Auftrag",
}
ALL_ERROR["80006"] = {
    "error_code": "80006",
    "error_msg": "Invalid place workspace_id",
    "zh_msg": "任务错误：WCS下发放置工作空间不存在，任务无法进行",
    "tip": "WCS下发的任务订单错误，请发送正确的任务订单",
    "ja_msg": "タスクエラー：WCSによって放置空間がないタスクが送信され、タスクを実行できません",
    "ja_tip": "WCSが送信したタスクオーダーが誤っているため、正しいタスクオーダーを送信してください",
    "en_msg": "Task error: The workspace issued by WCS does not exist, and the task cannot be performed",
    "en_tip": "The task order issued by WCS is wrong, please send the correct task order",
    "de_msg": "Fehler bei der Aufgabe: WCS verteilter Arbeitsbereich existiert nicht, die Aufgabe kann nicht ausgeführt werden",
    "de_tip": "Der von WCS herausgegebene Arbeitsauftrag ist falsch. Bitte senden Sie den richtigen Auftrag",
}
ALL_ERROR["80007"] = {
    "error_code": "80007",
    "error_msg": "The communication between xyz-hmi-back and WCS is broken",
    "zh_msg": "工控机与WCS通信断连",
    "tip": "请检查工控机与WCS之间的通信",
    "ja_msg": "IPC と WCS 間の通信信が切断されました",
    "ja_tip": "IPC と WCS 間の通信を確認してください",
    "en_msg": "The communication between the industrial computer and the WCS is disconnected",
    "en_tip": "Please check the communication between IPC and WCS",
    "de_msg": "Kommunikation zwischen Industrie-PC und WCS unterbrochen",
    "de_tip": "Bitte überprüfen Sie die Kommunikation zwischen IPC und WCS",
}
ALL_ERROR["80008"] = {
    "error_code": "80008",
    "error_msg": "Sku size error",
    "zh_msg": "SKU不符合技术指标",
    "tip": "WCS下发的任务订单错误，请发送正确的任务订单",
    "ja_msg": "SKUが技術仕様に適合していない",
    "ja_tip": "WCSが送信したタスクオーダーが誤っているため、正しいタスクオーダーを送信してください",
    "en_msg": "SKU does not meet specifications",
    "en_tip": "The task order issued by WCS is wrong, please send the correct task order",
    "de_msg": "SKU entspricht nicht den technischen Anforderungen",
    "de_tip": "Der von WCS herausgegebene Arbeitsauftrag ist falsch. Bitte senden Sie den richtigen Auftrag",
}
ALL_ERROR["80009"] = {
    "error_code": "80009",
    "error_msg": "The task is finished, please change the pallet or tote",
    "zh_msg": "任务完成，请更换托盘或料箱",
    "tip": "任务完成，请更换托盘或料箱",
    "ja_msg": "タスク完了、トレイやビンを交換してください",
    "ja_tip": "タスク完了、トレイやビンを交換してください",
    "en_msg": "The task is completed, please replace the pallet or bin",
    "en_tip": "The task is completed, please replace the pallet or bin",
    "de_msg": "Die Aufgabe ist abgeschlossen, bitte tauschen Sie die Palette oder die Lagerkisten aus",
    "de_tip": "Der Auftrag ist abgeschlossen, bitte tauschen Sie die Palette oder die Lagerkisten aus",
}
ALL_ERROR["80010"] = {
    "error_code": "80010",
    "error_msg": "Not enough objects",
    "zh_msg": "箱数不足",
    "tip": "1）请检查WCS下发的信息是否正确，请检查来料是否正确；2）请根据现场实际，WCS取消订单，修改尺寸信息后重新下发；3）请参考HMI用户手册，根据现场实际情况，进行处理；4）请判断是否要进行转人工处理；5）若未解决，请联系售后人员",
    "ja_msg": "箱数が足りない",
    "ja_tip": "1）WCSから送信された情報が正しいかどうか、来たSKUが正しいかどうか確認してください；2）現場の実際状況に従って、WCSがオーダーをキャンセルし、サイズ情報を変更した後、再度送信してください；3）HMIのユーザーマニュアルをご参照の上、現場の実情に合わせてお取り扱いください。；4）手動処理を行うかどうか判断してください；5）解決されない場合は、アフターサービス担当までご連絡ください",
    "en_msg": "Not enough objects",
    "en_tip": "1) Please check whether the information issued by WCS is correct, and please check whether the incoming materials are correct; 2) Please cancel the order according to the actual situation on site, modify the size information and issue it again; 3) Please refer to the HMI user manual, according to the actual situation on site , and deal with it; 4) Please judge whether to transfer to manual processing; 5) If it is not resolved, please contact the after-sales personnel",
    "de_msg": "Unzureichende Kartons",
    "de_tip": "Bitte prüfen Sie, ob die von WCS herausgegebenen Informationen korrekt sind und ob die eingehenden Materialien richtig sind; 2) Entsprechend der tatsächlichen Situation vor Ort storniert WCS den Auftrag, ändert die Größenangaben und gibt ihn erneut heraus; 3) Bitte lesen Sie das HMI-Benutzerhandbuch und handhaben Sie es entsprechend der tatsächlichen Situation vor Ort; 4) Bitte beurteilen Sie, ob Sie auf manuelle Bearbeitung umstellen müssen; 5) Wenn nicht, wenden Sie sich bitte an den Kundendienst",
}
ALL_ERROR["80011"] = {
    "error_code": "80011",
    "error_msg": "The carton on the conveyor is not in place, time out",
    "zh_msg": "输送线箱子不到位，超时",
    "tip": "输送线上的纸箱到位信号未到，请检查是否有纸箱，请检查光电传感器设备是否有故障，请检查工控机与PLC是否断连",
    "ja_msg": "コンベアライン箱が所定の位置にない、タイムアウト",
    "ja_tip": "コンベアラインの所定の位置に箱の信号が到着していない、箱があるかどうか確認してください、光電センサが故障しているかどうか確認してください、IPCがPLCから切断されているかどうか確認してください",
    "en_msg": "Conveyor line box is not in place, time out",
    "en_tip": "The carton in-position signal on the conveyor line has not arrived, please check whether there is a carton, please check whether the photoelectric sensor equipment is faulty, please check whether the industrial computer is disconnected from the PLC",
    "de_msg": "Kasten auf das Fliessband nicht an ihrem Platz, Zeitüberschreitung",
    "de_tip": 'Das Signal "Karton vorhanden" an der Förderstrecke ist noch nicht eingetroffen. Bitte prüfen Sie, ob Kartons vorhanden sind, ob der optoelektronische Sensor defekt ist und ob IPC und SPS nicht miteinander verbunden sind.',
}
ALL_ERROR["80012"] = {
    "error_code": "80012",
    "error_msg": "The sku is overweight",
    "zh_msg": "WCS下发的纸箱：超重",
    "tip": "1）请检查WCS下发的信息是否正确，请检查来料是否正确；2）请根据现场实际，WCS取消订单，修改尺寸信息后重新下发；3）请参考HMI用户手册，根据现场实际情况，进行处理；4）请判断是否要进行转人工处理；5）若未解决，请联系售后人员",
    "ja_msg": "SKU過重",
    "ja_tip": "1）WCSから送信された情報が正しいかどうか、来たSKUが正しいかどうか確認してください；2）現場の実際状況に従って、WCSがオーダーをキャンセルし、サイズ情報を変更した後、再度送信してください；3）HMIのユーザーマニュアルをご参照の上、現場の実情に合わせてお取り扱いください。；4）手動処理を行うかどうか判断してください；5）解決されない場合は、アフターサービス担当までご連絡ください",
    "en_msg": "Cartons issued by WCS: Overweight",
    "en_tip": "1) Please check whether the information issued by WCS is correct, and please check whether the incoming materials are correct; 2) Please cancel the order according to the actual situation on site, modify the size information and issue it again; 3) Please refer to the HMI user manual, according to the actual situation on site , and deal with it; 4) Please judge whether to transfer to manual processing; 5) If it is not resolved, please contact the after-sales personnel",
    "de_msg": "Vom WCS ausgegebener Karton: Übergewicht",
    "de_tip": "Bitte prüfen Sie, ob die von WCS herausgegebenen Informationen korrekt sind und ob die eingehenden Materialien richtig sind; 2) Entsprechend der tatsächlichen Situation vor Ort storniert WCS den Auftrag, ändert die Größenangaben und gibt ihn erneut heraus; 3) Bitte lesen Sie das HMI-Benutzerhandbuch und handhaben Sie es entsprechend der tatsächlichen Situation vor Ort; 4) Bitte beurteilen Sie, ob Sie auf manuelle Bearbeitung umstellen müssen; 5) Wenn nicht, wenden Sie sich bitte an den Kundendienst",
}
ALL_ERROR["80013"] = {
    "error_code": "80013",
    "error_msg": "The xyz-hmi-back cannot be connected",
    "zh_msg": "工控机后端无法连接",
    "tip": "请检查工控机后端是否打开，可以重启工控机，若问题未解决，请联系售后人员",
    "ja_msg": "IPCバックエンドが接続できない",
    "ja_tip": "IPCのバックエンドが開けるかどうか確認してください。IPCを再起動して、解決されない場合は、アフターサービス担当までご連絡ください",
    "en_msg": "The back end of the industrial computer cannot be connected",
    "en_tip": "Please check whether the back end of the industrial computer is open, you can restart the industrial computer, if the problem is not solved, please contact the after-sales personnel",
    "de_msg": "IPC-Backend kann keine Verbindung herstellen",
    "de_tip": "Prüfen Sie, ob das IPC-Backend geöffnet ist, und starten Sie den IPC neu. Wenn das Problem nicht behoben ist, wenden Sie sich bitte an den Kundendienst.",
}
ALL_ERROR["80014"] = {
    "error_code": "80014",
    "error_msg": "The xyz-hmi-back returns error",
    "zh_msg": "工控机后端返回结果错误",
    "tip": "请重启工控机后端程序，可以重启工控机，若问题未解决，请联系售后人员",
    "ja_msg": "IPCバックエンドリターン結果エラー",
    "ja_tip": "IPCを再起動して、解決されない場合は、アフターサービス担当までご連絡ください",
    "en_msg": "The back end of the industrial computer returns the wrong result",
    "en_tip": "Please restart the back-end program of the industrial computer, you can restart the industrial computer, if the problem is not solved, please contact the after-sales personnel",
    "de_msg": "Das vom hinteren Ende des Instrumentenbrettes (IPC) zurückgegebene Ergebnis ist falsch",
    "de_tip": "Bitte starten Sie das Back-End-Programm des IPC neu. Sie können die IPC neu starten. Wenn das Problem nicht behoben ist, wenden Sie sich bitte an den Kundendienst.",
}
ALL_ERROR["80015"] = {
    "error_code": "80015",
    "error_msg": "WCS send sku length is lesser than width.",
    "zh_msg": "WCS下发的sku的尺寸错误，长不能小于宽，请WCS取消任务，修改信息后，重新下发",
    "tip": "WCS下发的sku的尺寸错误，长不能小于宽，请WCS取消任务，修改信息后，重新下发",
    "ja_msg": "WCSから送られたskuのサイズが間違っている、長さが幅より小さいことはできない、WCSにタスクをキャンセルし、情報を変更して再送信するよう依頼してください。",
    "ja_tip": "WCSから送られたskuのサイズが間違っている、長さが幅より小さいことはできない、WCSにタスクをキャンセルし、情報を変更して再送信するよう依頼してください。",
    "en_msg": "The size of the sku sent by WCS is wrong. The length cannot be smaller than the width. Please cancel the task by WCS, modify the information, and send it again.",
    "en_tip": "The size of the sku sent by WCS is wrong. The length cannot be smaller than the width. Please cancel the task by WCS, modify the information, and send it again.",
    "de_msg": "Die vom WCS herausgegebene Sku-Größe ist falsch. Die Länge kann nicht kleiner als die Breite sein. Bitten Sie das WCS, den Auftrag abzubrechen, die Informationen zu ändern und erneut zu veröffentlichen.",
    "de_tip": "Die von WCS veröffentlichte Sku-Größe ist falsch. Die Länge kann nicht kleiner als die Breite sein. Bitten Sie WCS, den Auftrag abzubrechen, die Informationen zu ändern und sie erneut zu veröffentlichen.",
}
ALL_ERROR["80016"] = {
    "error_code": "80016",
    "error_msg": "The incoming signal of the conveyor line is not ready.",
    "zh_msg": "输送线来料信号未到位",
    "tip": "请检查输送线来料信号未到原因：1）来料未到；2）信号中断；3）传感器信息错误",
    "ja_msg": "コンベアライン入庫信号が届かない",
    "ja_tip": "コンベアラインからの信号が届かない理由をご確認ください：1) SKUが届かない、2) 信号が途切れる、3) センサー情報のエラー",
    "en_msg": "The incoming signal of the conveyor line is not in place",
    "en_tip": "Please check the reasons why the incoming signal of the conveyor line has not arrived: 1) the incoming material has not arrived; 2) the signal is interrupted; 3) the sensor information is wrong",
    "de_msg": "Das Eingangssignal der Förderstrecke ist nicht vorhanden",
    "de_tip": "Überprüfen Sie bitte die Gründe, warum das Fördersignal des Förderbandes nicht ankommt: 1) Das Zufuhrsignal kommt nicht an; 2) Signalunterbrechung; 3) Sensorinformationsfehler",
}
ALL_ERROR["80017"] = {
    "error_code": "80017",
    "error_msg": "The discharge signal of the conveyor line is not ready.",
    "zh_msg": "输送线允许放料信号未到位",
    "tip": "请确认传送带线排出信号未到达的原因：1）传送带线盒堵塞；2）信号中断；3）传感器信息错误。",
    "ja_msg": "请检查输送线放料信号未到原因：1）输送线堵箱；2）信号中断；3）传感器信息错误",
    "ja_tip": "コンベアライン排出信号が届いていない理由を確認してください：1）コンベアラインのボックスが詰まっている、2）信号が途切れている、3）センサー情報が間違っている。",
    "en_msg": "The permissive feeding signal of the conveyor line is not in place",
    "en_tip": "Please confirm the reasons why the discharge signal of the conveyor line does not arrive: 1) The conveyor line box is blocked; 2) Signal interruption; 3) Sensor information error.",
    "de_msg": "Das zulässige Zufuhrsignal der Förderstrecke ist nicht vorhanden",
    "de_tip": "Bitte überprüfen Sie die Gründe, warum das Entladesignal des Förderbandes nicht ankommt: 1) Der Förderbandkasten ist blockiert; 2) Signalunterbrechung; 3) Sensorinformationsfehler.",
}
ALL_ERROR["99995"] = {
    "error_code": "99995",
    "error_msg": "Vision dimension cannot pass dimension check",
    "zh_msg": "视觉识别误差大",
    "tip": "请联系售后人员",
    "ja_msg": "大きな視覚認識エラー",
    "ja_tip": "アフターサービス担当までご連絡ください",
    "en_msg": "Large error in visual recognition",
    "en_tip": "Please contact the after-sales personnel",
    "de_msg": "Große Fehler in der visuellen Erkennung",
    "de_tip": "Bitte wenden Sie sich an das Kundendienstpersonal",
}
ALL_ERROR["99996"] = {
    "error_code": "99996",
    "error_msg": "remote helper is not found",
    "zh_msg": "remote helper 发生异常",
    "tip": "请联系售后人员",
    "ja_msg": "remote helper 異常",
    "ja_tip": "アフターサービス担当までご連絡ください",
    "en_msg": "Remote helper exception",
    "en_tip": "Please contact the after-sales personnel",
    "de_msg": "Remote Helper-Ausnahme",
    "de_tip": "Bitte wenden Sie sich an das Kundendienstpersonal",
}
ALL_ERROR["99997"] = {
    "error_code": "99997",
    "error_msg": "Appointed workspace does not exist in environment",
    "zh_msg": "环境中不存在目标工作空间",
    "tip": "请联系售后人员",
    "ja_msg": "指定されたワークスペースが環境に存在しない",
    "ja_tip": "アフターサービス担当までご連絡ください",
    "en_msg": "The target workspace does not exist in the environment",
    "en_tip": "Please contact the after-sales personnel",
    "de_msg": "Der Zielarbeitsbereich ist in der Umgebung nicht vorhanden",
    "de_tip": "Bitte wenden Sie sich an das Kundendienstpersonal",
}
ALL_ERROR["99998"] = {
    "error_code": "99998",
    "error_msg": "program inner error",
    "zh_msg": "未定义的异常",
    "tip": "请联系售后人员",
    "ja_msg": "未定義のエラー",
    "ja_tip": "アフターサービス担当までご連絡ください",
    "en_msg": "Unknown exception",
    "en_tip": "Please contact the after-sales personnel",
    "de_msg": "Undefinierte Ausnahme",
    "de_tip": "Bitte wenden Sie sich an das Kundendienstpersonal",
}
ALL_ERROR["E0000"] = {
    "error_code": "E0000",
    "error_msg": "Start task failure",
    "zh_msg": "任务开始失败",
    "tip": "请联系售后人员",
    "ja_msg": "タスク開始失敗",
    "ja_tip": "アフターサービス担当までご連絡ください",
    "en_msg": "Task execution failed",
    "en_tip": "Please contact the after-sales staff",
    "de_msg": "Task-Start fehlgeschlagen",
    "de_tip": "Bitte wenden Sie sich an das Kundendienstpersonal",
}
ALL_ERROR["E0001"] = {
    "error_code": "E0001",
    "error_msg": "Stop running task failure for something blocking the task",
    "zh_msg": "任务停止失败",
    "tip": "请联系售后人员",
    "ja_msg": "タスク停止失敗",
    "ja_tip": "アフターサービス担当までご連絡ください",
    "en_msg": "Task stop failed",
    "en_tip": "Please contact the after-sales staff",
    "de_msg": "Taskstopp fehlgeschlagen",
    "de_tip": "Bitte wenden Sie sich an das Kundendienstpersonal",
}
ALL_ERROR["E0002"] = {
    "error_code": "E0002",
    "error_msg": "Pause running task failure for something blocking the task",
    "zh_msg": "任务暂停失败",
    "tip": "请联系售后人员",
    "ja_msg": "タスク一時停止失敗",
    "ja_tip": "アフターサービス担当までご連絡ください",
    "en_msg": "Task pause failed",
    "en_tip": "Please contact the after-sales staff",
    "de_msg": "Task-Unterbrechung fehlgeschlagen",
    "de_tip": "Bitte wenden Sie sich an das Kundendienstpersonal",
}
ALL_ERROR["E0003"] = {
    "error_code": "E0003",
    "error_msg": "Invald task path that we cannot start",
    "zh_msg": "项目工程路径错误",
    "tip": "请联系售后人员",
    "ja_msg": "プロジェクト プロジェクト パス エラー",
    "ja_tip": "アフターサービス担当までご連絡ください",
    "en_msg": "Project project path error",
    "en_tip": "Please contact the after-sales staff",
    "de_msg": "Fehler im Projektpfad",
    "de_tip": "Bitte wenden Sie sich an das Kundendienstpersonal",
}
ALL_ERROR["E0004"] = {
    "error_code": "E0004",
    "error_msg": "Continue running task  failure",
    "zh_msg": "任务继续执行失败",
    "tip": "请联系售后人员",
    "ja_msg": "タスクの継続実行の失敗",
    "ja_tip": "アフターサービス担当までご連絡ください",
    "en_msg": "Task continued execution failed",
    "en_tip": "Please contact the after-sales staff",
    "de_msg": "Fortgesetzte Ausführung der Aufgabe fehlgeschlagen",
    "de_tip": "Bitte wenden Sie sich an das Kundendienstpersonal",
}
ALL_ERROR["E0005"] = {
    "error_code": "E0005",
    "error_msg": "Initialize nodes failure",
    "zh_msg": "初始化节点失败",
    "tip": "请联系售后人员",
    "ja_msg": "ノードの初期化失敗",
    "ja_tip": "アフターサービス担当までご連絡ください",
    "en_msg": "Failed to initialize node",
    "en_tip": "Please contact the after-sales staff",
    "de_msg": "Initialisierung des Knotens fehlgeschlagen",
    "de_tip": "Bitte wenden Sie sich an das Kundendienstpersonal",
}
ALL_ERROR["E0006"] = {
    "error_code": "E0006",
    "error_msg": "Robot is not in home pose",
    "zh_msg": "机器人不在初始点",
    "tip": "1）机器人切手动，将机器人移动到初始点附近，然后再切回自动；2）重启机器人节点，再次连接；3）请参考HMI用户手册，根据现场实际情况，继续当前任务",
    "ja_msg": "アームロボットは初期位置ではない",
    "ja_tip": "アームロボットは手動モードで制御し、初期位置の近くまで移動してください",
    "en_msg": "The robot is not at the initial point",
    "en_tip": "1) The robot switches to manual mode, moves the robot near the initial point, and then switches back to automatic mode; 2) Restart the robot node and connect it again; 3) Please refer to the HMI user manual and continue the current task according to the actual situation",
    "de_msg": "Der Roboter befindet sich nicht am Anfangspunkt",
    "de_tip": "Der Roboter wechselt in den manuellen Modus, bewegt sich in die Nähe des Anfangspunkts und wechselt dann zurück in den automatischen Modus; 2) Starten Sie den Roboterknoten neu und stellen Sie die Verbindung wieder her; 3) Lesen Sie im HMI-Benutzerhandbuch nach und setzen Sie die aktuelle Aufgabe entsprechend der tatsächlichen Situation vor Ort fort",
}
ALL_ERROR["E0100"] = {
    "error_code": "E0100",
    "error_msg": "Robot is in collision",
    "zh_msg": "机器人检测到碰撞",
    "tip": "1）机器人转手动，处理当前错误异常；2）判断是否为sku尺寸信息错误；3）若发生掉箱，请参考HMI用户手册处理；4）若处理不了，请联系售后人员",
    "ja_msg": "アームロボットが衝突を検出しました",
    "ja_tip": "1）手動でエラーを処理する；2）skuサイズ情報エラーかどうかを判断する；3）落下が発生した場合、HMIユーザーマニュアルを参照し処理する；4）処理できない場合、アフターサービス担当までご連絡ください",
    "en_msg": "The robot detects a collision",
    "en_tip": "1) The robot switches to manual mode to handle the current error exception; 2) Judge whether it is sku size information error; 3) In case of box drop, please refer to HMI user manual for handling; 4) If the problem cannot be solved, please contact the after-sales service personnel",
    "de_msg": "Der Roboter hat eine Kollision festgestellt",
    "de_tip": "Der Roboter schaltet in den manuellen Modus, um die aktuelle Fehlerausnahme zu behandeln; 2) Stellen Sie fest, ob die Informationen zur Sku-Größe falsch sind; 3) Wenn die Box herunterfällt, lesen Sie bitte im HMI-Benutzerhandbuch nach; 4) Wenn das Problem nicht gelöst werden kann, wenden Sie sich bitte an das Kundendienstpersonal",
}
ALL_ERROR["E0101"] = {
    "error_code": "E0101",
    "error_msg": "Robot exceeds joint limit",
    "zh_msg": "机器人超过关节限制",
    "tip": "请联系售后人员",
    "ja_msg": "ロボットが関節制限を超えた ",
    "ja_tip": " アフターサービス担当までご連絡ください",
    "en_msg": "Robot exceeds joint limit",
    "en_tip": "Please contact the after-sales staff",
    "de_msg": "Roboter überschreitet Gelenkgrenze",
    "de_tip": "Bitte wenden Sie sich an das Kundendienstpersonal",
}
ALL_ERROR["E0102"] = {
    "error_code": "E0102",
    "error_msg": "Robot exceeds payload limit",
    "zh_msg": "机器人超过负载限制",
    "tip": "1）机器人转手动，处理当前错误异常；2）判断物体是否超过技术协议约定的重量；3）若处理不了，请联系售后人员",
    "ja_msg": "ロボットがペイロードの上限を超えた",
    "ja_tip": "1）手動でエラーを処理する；2）技術契約で合意された重量を超えたかどうかを判断する；3）処理できない場合、アフターサービス担当までご連絡ください",
    "en_msg": "Robot exceeds payload limit",
    "en_tip": "1) The robot switches to manual mode to handle the current error exception; 2) Judge whether the object exceeds the weight agreed in the technical agreement; 3) If the problem cannot be solved, please contact the after-sales service personnel",
    "de_msg": "Roboter überschreitet Lastgrenze",
    "de_tip": "Der Roboter schaltet in den manuellen Modus, um die aktuelle Fehlerausnahme zu behandeln; 2) Beurteilen Sie, ob das Objekt das in der technischen Vereinbarung vereinbarte Gewicht überschreitet; 3) Wenn das Problem nicht gelöst werden kann, wenden Sie sich bitte an das Kundendienstpersonal",
}
ALL_ERROR["E0103"] = {
    "error_code": "E0103",
    "error_msg": "Robot is in emergency stop state",
    "zh_msg": "机器人急停",
    "tip": "1）異常処理後、ロボットの緊急停止を解除する。 1）请先处理完异常后，解除机器人急停，解除方法是先将红色急停按钮旋转弹起，再按下电控柜上的复位按钮；2）重启机器人节点，再次连接；3）请参考HMI用户手册，根据现场实际情况，选择继续当前任务或取消任务，转人工处理",
    "ja_msg": "アームロボットが緊急停止状態です",
    "ja_tip": "現場の故障を解消し、非常停止を解除した後、アームロボットがをリセットしてください！",
    "en_msg": "The robot has an emergency stop",
    "en_tip": "1) After the abnormality is handled, the emergency stop of the robot is canceled. 1) After handling the abnormality first, release the emergency stop of the robot. The method of release is to rotate the red emergency stop button and then press the reset button on the electric control cabinet; 2) Restart the robot node and connect again; 3) Please refer to HMI user manual, according to the actual situation on site, choose to continue the current task or cancel the task, and transfer to manual processing",
    "de_msg": "Not-Aus des Roboters",
    "de_tip": "Nachdem Sie die Anomalie behoben haben, entfernen Sie bitte die Not-Aus-Vorrichtung des Roboters. 2) Starten Sie den Roboterknoten neu und schließen Sie ihn wieder an; 3) Lesen Sie das HMI-Benutzerhandbuch und wählen Sie je nach der tatsächlichen Situation vor Ort, ob Sie die aktuelle Aufgabe fortsetzen oder die Aufgabe abbrechen und zur manuellen Bearbeitung wechseln möchten",
}
ALL_ERROR["E0200"] = {
    "error_code": "E0200",
    "error_msg": "The safety gate is open now! Please close the gate before starting!",
    "zh_msg": "安全门被打开，请在开始运行前先关闭安全门！",
    "tip": "1）请先处理完异常后，关闭安全门；2）重启机器人节点，再次连接；3）请参考HMI用户手册，根据现场实际情况，选择继续当前任务或取消任务，转人工处理",
    "ja_msg": "安全ドアが開いているので、操作を始める前に閉めてください！",
    "ja_tip": "1）異常処理を先に済ませてから、安全扉を閉めてください；2）ロボットノードを再起動し、再度接続する；3）HMIのユーザーマニュアルを参照し、現場の実情に応じて、現在のタスクを継続するか、キャンセルするかを選択してください。手動に切り替えること",
    "en_msg": "The safety door is opened, please close it before starting operation!",
    "en_tip": "1) Please close the safety door after handling the abnormality; 2) Restart the robot node and connect it again; 3) Please refer to the HMI user manual and choose to continue the current task or cancel the task and transfer to manual processing according to the actual situation",
    "de_msg": "Die Schutztür wurde geöffnet, bitte schließen Sie sie, bevor Sie den Betrieb starten!",
    "de_tip": "Schließen Sie die Schutztür, nachdem Sie die Störung behoben haben; 2) Starten Sie den Roboterknoten neu und stellen Sie die Verbindung wieder her; 3) Lesen Sie im HMI-Benutzerhandbuch nach und wählen Sie, ob Sie die aktuelle Aufgabe fortsetzen oder abbrechen und zur manuellen Bearbeitung wechseln möchten, je nach der tatsächlichen Situation vor Ort",
}
ALL_ERROR["E0500"] = {
    "error_code": "E0500",
    "error_msg": " Object is dropped",
    "zh_msg": "检测到发生掉箱，物体掉落",
    "tip": "1）请参考HMI用户手册处理掉箱异常及恢复；2）请根据现场实际情况，机器人拍急停，处理掉箱异常，注意安全操作规范；3）若未解决，请联系售后人员",
    "ja_msg": "物体ドロップを検知",
    "ja_tip": "1）HMIのユーザーマニュアルを参照し、処理してください。；2）現場の状況に応じてロボットを緊急停止して、箱落下の異常に対処し、安全操作の規範に注意してください。；3）解決されない場合は、アフターサービス担当までご連絡ください",
    "en_msg": "Box drop is detected and objects fall",
    "en_tip": "1) Please refer to the HMI user manual to handle the case drop exception and restore; 2) According to the actual situation on site, the robot should make an emergency stop, handle the abnormal case dropping, and pay attention to the safety operation specification; 3) If not, please contact after-sales personnel",
    "de_msg": "Es wird festgestellt, dass die Box fällt und das Objekt fällt",
    "de_tip": "Beziehen Sie sich bitte auf das HMI-Benutzerhandbuch, um Ausnahmen und die Wiederherstellung des Falles zu handhaben; 2) Entsprechend der tatsächlichen Situation vor Ort muss der Roboter in einem Notfall anhalten, den anormalen Fall eines Sturzes handhaben und die Spezifikationen für den Sicherheitsbetrieb beachten; 3) Falls nicht, wenden Sie sich bitte an das Kundendienstpersonal",
}
ALL_ERROR["E0501"] = {
    "error_code": "E0501",
    "error_msg": "There is something on the tool changer",
    "zh_msg": "吸盘更换机构上有异物",
    "tip": "1）请排查快换信号检测或控制异常；2）请参考HMI用户手册，根据现场实际情况，选择继续当前任务或取消任务，转人工处理",
    "ja_msg": "ツールチェンジャーに何か付いている",
    "ja_tip": "1）クイックチェンジ信号の検出または制御の異常を調査してください。2）HMIのユーザーマニュアルを参照し、現場の状況に応じて、現在のタスクを継続するか、キャンセルしてマニュアル処理に切り替えるかを選択してください",
    "en_msg": "Foreign matters on the suction cup replacement mechanism",
    "en_tip": "1) Please check whether the quick change signal detection or control is abnormal; 2) Please refer to the HMI user manual and choose to continue the current task or cancel the task and transfer to manual processing according to the actual situation",
    "de_msg": "Fremdkörper auf dem Austauschmechanismus des Saugers",
    "de_tip": "Überprüfen Sie, ob die Erkennung oder Steuerung des Schnellwechselsignals anormal ist; 2) Schlagen Sie im HMI-Benutzerhandbuch nach und wählen Sie je nach der tatsächlichen Situation vor Ort, ob Sie die aktuelle Aufgabe fortsetzen oder abbrechen und zur manuellen Bearbeitung wechseln möchten",
}
ALL_ERROR["E0502"] = {
    "error_code": "E0502",
    "error_msg": "Object is oversize",
    "zh_msg": "物体尺寸不匹配",
    "tip": "1）请检查WCS下发的信息是否正确，请检查来料是否正确；2）请根据现场实际，WCS取消订单，修改尺寸信息后重新下发；3）请参考HMI用户手册，根据现场实际情况，进行处理；4）请判断是否要进行转人工处理；5）若未解决，请联系售后人员",
    "ja_msg": "物体サイズが一致しない",
    "ja_tip": "1）WCSから発行された情報が正しいかどうか、入庫SKUが正しいかどうかご確認ください。2）WCSがキャンセルし、実際の現場に合わせてサイズ情報を修正した上で再発行してください。3）HMIのユーザーマニュアルを参考に、実際の現場の状況に合わせて処理してください。4）マニュアル処理への移行を実施するかどうかご判断をお願いします。5）解決されない場合は、アフターサービス担当までご連絡ください",
    "en_msg": "Object size mismatch",
    "en_tip": "1) Please check whether the information issued by WCS is correct and whether the incoming materials are correct; 2) According to the actual situation on site, WCS should cancel the order, modify the size information and re issue it; 3) Please refer to the HMI user manual and handle according to the actual situation on site; 4) Please judge whether to transfer to manual processing; 5) If not, please contact after-sales personnel",
    "de_msg": "Objektgröße stimmt nicht überein",
    "de_tip": "Bitte prüfen Sie, ob die von WCS herausgegebenen Informationen korrekt sind und ob die eingehenden Materialien richtig sind; 2) Entsprechend der tatsächlichen Situation vor Ort storniert WCS den Auftrag, ändert die Größenangaben und gibt ihn erneut heraus; 3) Bitte lesen Sie das HMI-Benutzerhandbuch und handhaben Sie es entsprechend der tatsächlichen Situation vor Ort; 4) Bitte beurteilen Sie, ob Sie auf manuelle Bearbeitung umstellen müssen; 5) Wenn nicht, wenden Sie sich bitte an den Kundendienst",
}
ALL_ERROR["E0503"] = {
    "error_code": "E0503",
    "error_msg": "scanner_barcode_failed",
    "zh_msg": "无法读取条码，可能因为条码破损或者打印质量偏低",
    "tip": "请参考HMI用户手册，根据现场实际情况，选择继续当前任务或取消任务，转人工处理",
    "ja_msg": "バーコードが読み取れない。バーコードが壊れているか、印刷品質が低い可能性がある。",
    "ja_tip": "1）WCSから発行された情報が正しいかどうか、入庫SKUが正しいかどうかご確認ください。2）WCSがキャンセルし、実際の現場に合わせてサイズ情報を修正した上で再発行してください。3）HMIのユーザーマニュアルを参考に、実際の現場の状況に合わせて処理してください。4）マニュアル処理への移行を実施するかどうかご判断をお願いします。HMIのユーザーマニュアルを参照し、現場の状況に応じて、現在のタスクを継続するか、キャンセルしてマニュアル処理に切り替えるかを選択してください",
    "en_msg": "Unable to read the barcode, possibly because the barcode is damaged or the print quality is low",
    "en_tip": "Please refer to the HMI user manual and choose to continue the current task or cancel the task and transfer to manual processing according to the actual situation",
    "de_msg": "Der Barcode kann nicht gelesen werden, möglicherweise weil der Barcode beschädigt ist oder die Druckqualität schlecht ist.",
    "de_tip": "Bitte lesen Sie das HMI-Benutzerhandbuch und wählen Sie je nach der tatsächlichen Situation vor Ort, ob Sie die aktuelle Aufgabe fortsetzen oder abbrechen und zur manuellen Bearbeitung wechseln möchten",
}
ALL_ERROR["E0504"] = {
    "error_code": "E0504",
    "error_msg": "Object is overweight",
    "zh_msg": "物体重量不匹配",
    "tip": "1）请检查WCS下发的信息是否正确，请检查来料是否正确；2）请根据现场实际，WCS取消订单，修改尺寸信息后重新下发；3）请参考HMI用户手册，根据现场实际情况，进行处理；4）请判断是否要进行转人工处理；5）若未解决，请联系售后人员",
    "ja_msg": "物体重量が一致しない",
    "ja_tip": "1）WCSから発行された情報が正しいかどうか、入庫SKUが正しいかどうかご確認ください。2）WCSがキャンセルし、実際の現場に合わせてサイズ情報を修正した上で再発行してください。3）HMIのユーザーマニュアルを参考に、実際の現場の状況に合わせて処理してください。4）マニュアル処理への移行を実施するかどうかご判断をお願いします。5）解決されない場合は、アフターサービス担当までご連絡ください",
    "en_msg": "Object weight mismatch",
    "en_tip": "1) Please check whether the information issued by WCS is correct and whether the incoming materials are correct; 2) According to the actual situation on site, WCS should cancel the order, modify the size information and re issue it; 3) Please refer to the HMI user manual and handle according to the actual situation on site; 4) Please judge whether to transfer to manual processing; 5) If not, please contact after-sales personnel",
    "de_msg": "Objektgewicht stimmt nicht überein",
    "de_tip": "Bitte prüfen Sie, ob die von WCS herausgegebenen Informationen korrekt sind und ob die eingehenden Materialien richtig sind; 2) Entsprechend der tatsächlichen Situation vor Ort storniert WCS den Auftrag, ändert die Größenangaben und gibt ihn erneut heraus; 3) Bitte lesen Sie das HMI-Benutzerhandbuch und handhaben Sie es entsprechend der tatsächlichen Situation vor Ort; 4) Bitte beurteilen Sie, ob Sie auf manuelle Bearbeitung umstellen müssen; 5) Wenn nicht, wenden Sie sich bitte an den Kundendienst",
}
ALL_ERROR["E0600"] = {
    "error_code": "E0600",
    "error_msg": "workspace is not empty but there is no primitive",
    "zh_msg": "工作空间点云不为空但视觉异常",
    "tip": "请联系售后人员",
    "ja_msg": "ワークスペース点群は空ではないが視覚的に異常がある",
    "ja_tip": "アフターサービス担当までご連絡ください",
    "en_msg": "The workspace point cloud is not empty, but the vision is abnormal",
    "en_tip": "Please contact after-sales personnel",
    "de_msg": "Die Punktwolke im Arbeitsbereich ist nicht leer, aber die Bildverarbeitung ist anormal",
    "de_tip": "Bitte wenden Sie sich an das Kundendienstpersonal",
}
ALL_ERROR["E0601"] = {
    "error_code": "E0601",
    "error_msg": "Start picking vision error",
    "zh_msg": "抓取视觉启动失败",
    "tip": "请断开相机开关电源，重新插拔相机网线，重启相机，再进行测试，如未恢复，请联系售后人员",
    "ja_msg": "ビジョン起動失敗",
    "ja_tip": "カメラのスイッチの電源を切り、カメラのネットワークケーブルを再度接続し、カメラを再起動してから、再度テストしてください。解決されない場合、アフターサービス担当までご連絡ください",
    "en_msg": "Failed to start grabbing vision",
    "en_tip": "Please disconnect the camera switch power supply, re plug the camera network cable, restart the camera, and test again. If it is not recovered, please contact the after-sales service personnel",
    "de_msg": "Bildverarbeitung von Greifen konnte nicht gestartet werden",
    "de_tip": "Bitte unterbrechen Sie die Stromversorgung der Kamera, stecken Sie das Netzwerkkabel der Kamera wieder ein, starten Sie die Kamera neu und testen Sie sie dann erneut. Wenn das Problem nicht behoben werden kann, wenden Sie sich bitte an den Kundendienst.",
}
ALL_ERROR["E0602"] = {
    "error_code": "E0602",
    "error_msg": "Camera hardware error, need to reset camera",
    "zh_msg": "相机失效",
    "tip": "请断开相机开关电源，重新插拔相机网线，重启相机，再进行测试，如未恢复，请联系售后人员",
    "ja_msg": "カメラの故障",
    "ja_tip": "カメラのスイッチの電源を切り、カメラのネットワークケーブルを再度接続し、カメラを再起動してから、再度テストしてください。解決されない場合、アフターサービス担当までご連絡ください",
    "en_msg": "Camera failure",
    "en_tip": "Please disconnect the camera switch power supply, re plug the camera network cable, restart the camera, and test again. If it is not recovered, please contact the after-sales service personnel",
    "de_msg": "Kamerafehler",
    "de_tip": "Bitte unterbrechen Sie die Stromversorgung der Kamera, stecken Sie das Netzwerkkabel der Kamera wieder ein, starten Sie die Kamera neu und testen Sie sie dann erneut. Wenn das Problem nicht behoben werden kann, wenden Sie sich bitte an den Kundendienst.",
}
ALL_ERROR["E0603"] = {
    "error_code": "E0603",
    "error_msg": "Camera view error, need to recalibrate camera",
    "zh_msg": "相机标定结果失效",
    "tip": "请联系售后人员",
    "ja_msg": "カメラキャリブレーション結果が無効",
    "ja_tip": "アフターサービス担当までご連絡ください",
    "en_msg": "Camera calibration result is invalid",
    "en_tip": "Please contact the after-sales service personnel",
    "de_msg": "Das Ergebnis der Kamerakalibrierung ist ungültig",
    "de_tip": "Bitte wenden Sie sich an das Kundendienstpersonal",
}
ALL_ERROR["E0604"] = {
    "error_code": "E0604",
    "error_msg": "Learning error, need to restart learning node",
    "zh_msg": "深度学习结果失效",
    "tip": "重启工控机，如未解决，请联系售后人员",
    "ja_msg": "ディープラーニングの結果が無効",
    "ja_tip": "解決しない場合は、産業用コンピュータを再起動します、アフターサービス担当までご連絡ください",
    "en_msg": "Failure of deep learning results",
    "en_tip": "Restart the IPC, if not, please contact the after-sales service personnel",
    "de_msg": "Deep Learning-Ergebnis fehlerhaft",
    "de_tip": "Starten Sie den IPC neu, wenn nicht, wenden Sie sich bitte an den Kundendienst.",
}
ALL_ERROR["E0605"] = {
    "error_code": "E0605",
    "error_msg": "No enough background",
    "zh_msg": "没有足够的视觉背景",
    "tip": "请联系售后人员",
    "ja_msg": "ビジョン背景が足りない",
    "ja_tip": "アフターサービス担当までご連絡ください",
    "en_msg": "Not enough visual background",
    "en_tip": "Please contact the after-sales service personnel",
    "de_msg": "Unzureichender visueller Hintergrund",
    "de_tip": "Bitte wenden Sie sich an das Kundendienstpersonal",
}
ALL_ERROR["E0606"] = {
    "error_code": "E0606",
    "error_msg": "No images, please capture images first",
    "zh_msg": "没有拍摄图像",
    "tip": "请联系售后人员",
    "ja_msg": "画像なし",
    "ja_tip": "アフターサービス担当までご連絡ください",
    "en_msg": "Not enough visual background",
    "en_tip": "Please contact the after-sales service personnel",
    "de_msg": "Kein Bild aufgenommen",
    "de_tip": "Bitte wenden Sie sich an das Kundendienstpersonal",
}
ALL_ERROR["E0607"] = {
    "error_code": "E0607",
    "error_msg": "Start place vision error",
    "zh_msg": "放置视觉启动失败",
    "tip": "请断开相机开关电源，重新插拔相机网线，重启相机，再进行测试，如未恢复，请联系售后人员",
    "ja_msg": "スタート地点の視力エラー",
    "ja_tip": "カメラのスイッチの電源を切り、カメラのネットワークケーブルを再度接続し、カメラを再起動してから、再度テストしてください。解決されない場合、アフターサービス担当までご連絡ください",
    "en_msg": "Placing visual start failed",
    "en_tip": "Please disconnect the camera switch power supply, re plug the camera network cable, restart the camera, and test again. If it is not recovered, please contact the after-sales service personnel",
    "de_msg": "Bildverarbeitung vom Platzieren lässt sich nicht starten",
    "de_tip": "Bitte unterbrechen Sie die Stromversorgung der Kamera, stecken Sie das Netzwerkkabel der Kamera wieder ein, starten Sie die Kamera neu und testen Sie sie dann erneut. Wenn das Problem nicht behoben werden kann, wenden Sie sich bitte an den Kundendienst.",
}
ALL_ERROR["E0608"] = {
    "error_code": "E0608",
    "error_msg": "Vision result did not pass the final check",
    "zh_msg": "视觉结果校验错误",
    "tip": "1）请检查WCS下发的信息是否正确，请检查来料是否正确；2）请根据现场实际，WCS取消订单，修改尺寸信息后重新下发；3）请参考HMI用户手册，根据现场实际情况，进行处理；4）请判断是否要进行转人工处理；5）若未解决，请联系售后人员",
    "ja_msg": "視覚結果検証エラー",
    "ja_tip": "WCS が発行する情報が正しいかどうか、および入荷資料が正しいかどうかを確認してください; 2) WCS が注文をキャンセルした後、実際の現場で合意された異常の処理方法を参照し、注文を修正し、再度発行するか、手動処理に移行します; 3) HMI のユーザー マニュアルを参照し、現場の実際の状況に応じて処理してください; 4) 解決されない場合は、アフターセールス担当者に連絡してください",
    "en_msg": "Visual result verification error",
    "en_tip": "1) Please check whether the information issued by WCS is correct and whether the incoming materials are correct; 2) According to the actual situation on site, WCS should cancel the order, modify the size information and re issue it; 3) Please refer to the HMI user manual and handle according to the actual situation on site; 4) Please judge whether to transfer to manual processing; 5) If not, please contact after-sales personnel",
    "de_msg": "Fehler bei der Validierung des visuellen Ergebnisses",
    "de_tip": "Bitte prüfen Sie, ob die von WCS herausgegebenen Informationen korrekt sind und ob die eingehenden Materialien richtig sind; 2) Entsprechend der tatsächlichen Situation vor Ort storniert WCS den Auftrag, ändert die Größenangaben und gibt ihn erneut heraus; 3) Bitte lesen Sie das HMI-Benutzerhandbuch und handhaben Sie es entsprechend der tatsächlichen Situation vor Ort; 4) Bitte beurteilen Sie, ob Sie auf manuelle Bearbeitung umstellen müssen; 5) Wenn nicht, wenden Sie sich bitte an den Kundendienst",
}
ALL_ERROR["E0609"] = {
    "error_code": "E0609",
    "error_msg": "Vision result did not pass the dimension check",
    "zh_msg": "视觉尺寸校验错误",
    "tip": "1）请检查WCS下发的信息是否正确，请检查来料是否正确；2）请根据现场实际，WCS取消订单，修改尺寸信息后重新下发；3）请参考HMI用户手册，根据现场实际情况，进行处理；4）请判断是否要进行转人工处理；5）若未解决，请联系售后人员",
    "ja_msg": "視覚サイズ検証エラー",
    "ja_tip": "WCS が発行する情報が正しいかどうか、および入荷資料が正しいかどうかを確認してください; 2) WCS が注文をキャンセルした後、実際の現場で合意された異常の処理方法を参照し、注文を修正し、再度発行するか、手動処理に移行します; 3) HMI のユーザー マニュアルを参照し、現場の実際の状況に応じて処理してください; 4) 解決されない場合は、アフターセールス担当者に連絡してください",
    "en_msg": "Visual dimension verification error",
    "en_tip": "1) Please check whether the information issued by WCS is correct and whether the incoming materials are correct; 2) According to the actual situation on site, WCS should cancel the order, modify the size information and re issue it; 3) Please refer to the HMI user manual and handle according to the actual situation on site; 4) Please judge whether to transfer to manual processing; 5) If not, please contact after-sales personnel",
    "de_msg": "Fehler bei der Überprüfung der visuellen Dimension",
    "de_tip": "Bitte prüfen Sie, ob die von WCS herausgegebenen Informationen korrekt sind und ob die eingehenden Materialien richtig sind; 2) Entsprechend der tatsächlichen Situation vor Ort storniert WCS den Auftrag, ändert die Größenangaben und gibt ihn erneut heraus; 3) Bitte lesen Sie das HMI-Benutzerhandbuch und handhaben Sie es entsprechend der tatsächlichen Situation vor Ort; 4) Bitte beurteilen Sie, ob Sie auf manuelle Bearbeitung umstellen müssen; 5) Wenn nicht, wenden Sie sich bitte an den Kundendienst",
}
ALL_ERROR["E0610"] = {
    "error_code": "E0610",
    "error_msg": "Label helper error",
    "zh_msg": "标注工具出错",
    "tip": "请联系售后人员",
    "ja_msg": "マークツールエラー",
    "ja_tip": "アフターサービス担当までご連絡ください",
    "en_msg": "Error in marker post tool",
    "en_tip": "Please contact the after-sales service personnel",
    "de_msg": "Fehler im Bemaßungswerkzeug",
    "de_tip": "Bitte wenden Sie sich an das Kundendienstpersonal",
}
ALL_ERROR["E0611"] = {
    "error_code": "E0611",
    "error_msg": "The conveyor sensor is damaged or blocked",
    "zh_msg": "输送线传感器损坏或阻塞",
    "tip": "请重启整个工作站设备，若问题未解决，请联系售后人员",
    "ja_msg": "コンベアラインセンサーが損傷またはブロックされている",
    "ja_tip": "ワークステーション本体全体を再起動してください、解決しない場合は、アフターサービス担当までご連絡ください",
    "en_msg": "The conveyor line sensor is damaged or blocked",
    "en_tip": "Please restart the whole workstation equipment. If the problem is not solved, please contact the after-sales service personnel",
    "de_msg": "Der Sensor der Förderstrecke ist beschädigt oder blockiert",
    "de_tip": "Starten Sie die gesamte Workstation neu. Wenn das Problem nicht behoben ist, wenden Sie sich bitte an den Kundendienst.",
}
ALL_ERROR["E0700"] = {
    "error_code": "E0700",
    "error_msg": "Motion plan failed",
    "zh_msg": "运动规划失败",
    "tip": "请联系售后人员",
    "ja_msg": "モーションプランニング失敗",
    "ja_tip": " アフターサービス担当までご連絡ください",
    "en_msg": "Motion planning failed",
    "en_tip": "Please contact the after-sales service personnel",
    "de_msg": "Bewegungsplanung fehlgeschlagen",
    "de_tip": "Bitte wenden Sie sich an das Kundendienstpersonal",
}
ALL_ERROR["E0701"] = {
    "error_code": "E0701",
    "error_msg": "Motion plan timeout",
    "zh_msg": "运动规划超时",
    "tip": "请联系售后人员",
    "ja_msg": "モーションプランニングタイムアウト",
    "ja_tip": " アフターサービス担当までご連絡ください",
    "en_msg": "Motion planning timeout",
    "en_tip": "Please contact the after-sales service personnel",
    "de_msg": "Zeitüberschreitung bei der Bewegungsplanung",
    "de_tip": "Bitte wenden Sie sich an das Kundendienstpersonal",
}
ALL_ERROR["E0702"] = {
    "error_code": "E0702",
    "error_msg": "Motion plan uninitialized",
    "zh_msg": "运动规划未初始化",
    "tip": "请联系售后人员",
    "ja_msg": "モーションプランニングが初期化されていない",
    "ja_tip": "アフターサービス担当までご連絡ください",
    "en_msg": "Motion planning not initialized",
    "en_tip": "Please contact the after-sales service personnel",
    "de_msg": "Bewegungsplanung nicht initialisiert",
    "de_tip": "Bitte wenden Sie sich an das Kundendienstpersonal",
}
ALL_ERROR["E0703"] = {
    "error_code": "E0703",
    "error_msg": "Target pose invalid",
    "zh_msg": "目标点位置非法",
    "tip": "请联系售后人员",
    "ja_msg": "ターゲットポーズが無効",
    "ja_tip": "アフターサービス担当までご連絡ください",
    "en_msg": "Illegal target point position",
    "en_tip": "Please contact the after-sales service personnel",
    "de_msg": "Unzulässige Zielpunktposition",
    "de_tip": "Bitte wenden Sie sich an das Kundendienstpersonal",
}
ALL_ERROR["E0704"] = {
    "error_code": "E0704",
    "error_msg": "Same node repeated",
    "zh_msg": "运动节点异常，节点重复使用",
    "tip": "请联系售后人员",
    "ja_msg": "ノード再利用",
    "ja_tip": "アフターサービス担当までご連絡ください",
    "en_msg": "The motion node is abnormal, and the node is used repeatedly",
    "en_tip": "Please contact the after-sales service personnel",
    "de_msg": "Abnormaler Bewegungsknoten, wiederholte Verwendung",
    "de_tip": "Bitte wenden Sie sich an das Kundendienstpersonal",
}
ALL_ERROR["E0705"] = {
    "error_code": "E0705",
    "error_msg": "Target node does not exist in planner",
    "zh_msg": "运动节点异常，未添加结点",
    "tip": "请联系售后人员",
    "ja_msg": "ノードなし",
    "ja_tip": "アフターサービス担当までご連絡ください",
    "en_msg": "Abnormal motion node, no node added",
    "en_tip": "Please contact the after-sales service personnel",
    "de_msg": "Abnormaler Bewegungsknoten, kein Knoten hinzugefügt",
    "de_tip": "Bitte wenden Sie sich an das Kundendienstpersonal",
}
ALL_ERROR["E0706"] = {
    "error_code": "E0706",
    "error_msg": "Picked item id not in workspace",
    "zh_msg": "抓取物体不存在",
    "tip": "请联系售后人员",
    "ja_msg": "ピックアップ対象物が存在しない",
    "ja_tip": "アフターサービス担当までご連絡ください",
    "en_msg": "Grab object does not exist",
    "en_tip": "Please contact the after-sales service personnel",
    "de_msg": "Greifobjekt existiert nicht",
    "de_tip": "Bitte wenden Sie sich an das Kundendienstpersonal",
}
ALL_ERROR["E0800"] = {
    "error_code": "E0800",
    "error_msg": "Gripper self-check failed",
    "zh_msg": "夹具异常：检测或控制失败",
    "tip": "1）请排查夹具检测或控制异常；2）请参考HMI用户手册，根据现场实际情况，选择继续当前任务或取消任务，转人工处理",
    "ja_msg": "ハンド異常：検出または制御の故障",
    "ja_tip": "1）検出または制御の故障を確認してください。；2）HMIのユーザーマニュアルを参照し、現場の状況に応じて、現在のタスクを継続するか、キャンセルしてマニュアル処理に切り替えるかを選択してください",
    "en_msg": "Fixture abnormality: detection or control failure",
    "en_tip": "1) Please check whether the fixture detection or control is abnormal; 2) Please refer to the HMI user manual and choose to continue the current task or cancel the task and transfer to manual processing according to the actual situation",
    "de_msg": "Anomalie in der Vorrichtung: Messung oder Steuerungsfehler",
    "de_tip": "Überprüfen Sie, ob die Erkennung oder Steuerung der Vorrichtung nicht normal ist; 2) Lesen Sie das HMI-Benutzerhandbuch und wählen Sie je nach der tatsächlichen Situation vor Ort, ob Sie die aktuelle Aufgabe fortsetzen oder abbrechen und zur manuellen Bearbeitung wechseln möchten",
}
ALL_ERROR["E0801"] = {
    "error_code": "E0801",
    "error_msg": "pressure is too low",
    "zh_msg": "气压值过低",
    "tip": "请检查空压机阀门是否打开，气管是否漏气，管路是否有挤压折叠漏气，压力表是否正常",
    "ja_msg": "圧力が低すぎる",
    "ja_tip": "エアコンプレッサーのバルブが開いているか、エア配管に漏れがないか、配管が折れていないか、圧力計は正常かなどをご確認ください",
    "en_msg": "The air pressure is too low",
    "en_tip": "Please check whether the air compressor valve is opened, whether the air pipe leaks, whether the pipeline is squeezed and folded, and whether the pressure gauge is normal",
    "de_msg": "Luftdruck zu niedrig",
    "de_tip": "Prüfen Sie, ob das Ventil des Luftkompressors geöffnet ist, ob die Luftleitung undicht ist, ob die Leitung gequetscht und gefaltet ist und ob das Manometer normal ist.",
}
ALL_ERROR["E0802"] = {
    "error_code": "E0802",
    "error_msg": "pressure is too high",
    "zh_msg": "气压值过高",
    "tip": "请检查空压机预设气压值是否过大，压力表是否正常",
    "ja_msg": "圧力が高すぎる",
    "ja_tip": "コンプレッサーの設定空気圧が高すぎないか、圧力計は正常かご確認ください。",
    "en_msg": "The air pressure is too high",
    "en_tip": "Please check whether the air compressor valve is opened, whether the air pipe leaks, whether the pipeline is squeezed and folded, and whether the pressure gauge is normal",
    "de_msg": "Der Luftdruckwert ist zu hoch",
    "de_tip": "Prüfen Sie, ob der voreingestellte Luftdruck des Luftkompressors zu hoch ist und ob das Manometer normal ist.",
}
ALL_ERROR["E0803"] = {
    "error_code": "E0803",
    "error_msg": "Gripper io timeout",
    "zh_msg": "抓具IO超时",
    "tip": "1）请排查夹具检测或控制异常；2）请参考HMI用户手册，根据现场实际情况，选择继续当前任务或取消任务，转人工处理",
    "ja_msg": "グリッパIOのタイムアウト",
    "ja_tip": "1）ツールの検出や制御の異常がないかをご確認ください。；2）HMIのユーザーマニュアルを参照し、現場の状況に応じて、現在のタスクを継続するか、キャンセルしてマニュアル処理に切り替えるかを選択してください",
    "en_msg": "Grab IO timeout",
    "en_tip": "1) Please check whether the fixture detection or control is abnormal; 2) Please refer to the HMI user manual and choose to continue the current task or cancel the task and transfer to manual processing according to the actual situation",
    "de_msg": "Zeitüberschreitung bei dem IO-Signal von Vorrichtung",
    "de_tip": "Überprüfen Sie, ob die Erkennung oder Steuerung der Vorrichtung nicht normal ist; 2) Lesen Sie das HMI-Benutzerhandbuch und wählen Sie je nach der tatsächlichen Situation vor Ort, ob Sie die aktuelle Aufgabe fortsetzen oder abbrechen und zur manuellen Bearbeitung wechseln möchten",
}
ALL_ERROR["E0900"] = {
    "error_code": "E0900",
    "error_msg": "No box pose can be returned. Please check the pallet dimensions and the box dimensions.",
    "zh_msg": "自动生成垛型规划失败，垛型为空",
    "tip": "自动生成垛型规划失败，垛型为空，请检查输入的托盘及箱子尺寸，如未解决，请联系售后人员",
    "ja_msg": "自動パレットプランニングが失敗した、積み方が空である",
    "ja_tip": "トレイとケースのサイズを確認してください。解決しない場合は、アフターサービス担当までご連絡ください",
    "en_msg": "Failed to automatically generate the stack type planning, the stack type is empty",
    "en_tip": "Failed to automatically generate the stacking type planning. The stacking type is empty. Please check the entered pallet and box dimensions. If not, please contact the after-sales service personnel",
    "de_msg": "Automatische Generierung des Stapelmusterplans nicht möglich, das Stapelmuster ist leer",
    "de_tip": "Es kann nicht automatisch ein Stapelmusterplan erstellt werden. Stapelmuster ist leer. Bitte überprüfen Sie die eingegebenen Paletten- und Kartonmaße. Falls nicht, wenden Sie sich bitte an das Kundendienstpersonal.",
}
ALL_ERROR["E0901"] = {
    "error_code": "E0901",
    "error_msg": "Layout stability check failed",
    "zh_msg": "自动生成垛型规划失败，稳定性检查未通过",
    "tip": "自动生成垛型规划失败，稳定性检查未通过，请检查输入的托盘及箱子尺寸，如未解决，请联系售后人员",
    "ja_msg": "自動パレットプランニングが失敗した、安定性チェック不合格",
    "ja_tip": "トレイとケースのサイズを確認してください。解決しない場合は、アフターサービス担当までご連絡ください",
    "en_msg": "Failed to automatically generate the stacking plan, and the stability check failed",
    "en_tip": "Failed to automatically generate the stacking plan. The stability check failed. Please check the entered pallet and box dimensions. If not, please contact the after-sales service personnel",
    "de_msg": "Stapelmusterplan kann nicht automatisch erstellt werden, Stabilitätsprüfung fehlgeschlagen",
    "de_tip": "Der Stapelmusterplan konnte nicht automatisch erstellt werden. Stabilitätsprüfung fehlgeschlagen. Bitte überprüfen Sie die eingegebenen Paletten- und Kartonmaße. Falls nicht, wenden Sie sich bitte an das Kundendienstpersonal.",
}
ALL_ERROR["E0902"] = {
    "error_code": "E0902",
    "error_msg": "Place pose could cause part of some box out of the workspace",
    "zh_msg": "机器人放置物品的姿态超出工作空间",
    "tip": "请联系售后人员",
    "ja_msg": "ロボットが作業スペースを超える姿勢で物を置くこと",
    "ja_tip": "アフターサービス担当までご連絡ください",
    "en_msg": "The pose of the robot placing objects exceeds the workspace",
    "en_tip": "Please contact after-sales personnel",
    "de_msg": "Die Position des Roboters zum Platzieren des Objekts überschreitet den Arbeitsbereich",
    "de_tip": "Bitte wenden Sie sich an das Kundendienstpersonal",
}
ALL_ERROR["E9998"] = {
    "error_code": "E9998",
    "error_msg": "parameter error",
    "zh_msg": "参数设置错误",
    "tip": "请联系售后人员",
    "ja_msg": "パラメータ設定エラー",
    "ja_tip": "アフターサービス担当までご連絡ください",
    "en_msg": "Parameter error",
    "en_tip": "Please contact after-sales personnel",
    "de_msg": "Fehler bei der Parametereinstellung",
    "de_tip": "Bitte wenden Sie sich an das Kundendienstpersonal",
}
ALL_ERROR["E9999"] = {
    "error_code": "E9999",
    "error_msg": "program inner error",
    "zh_msg": "程序内部错误",
    "tip": "请联系售后人员",
    "ja_msg": "プロジェクト内部エラー",
    "ja_tip": "アフターサービス担当までご連絡ください",
    "en_msg": "Internal error",
    "en_tip": "Please contact after-sales personnel",
    "de_msg": "Programminterner Fehler",
    "de_tip": "Bitte wenden Sie sich an das Kundendienstpersonal",
}
ALL_ERROR["99999"] = {
    "error_code": "99999",
    "error_msg": "The task planning flow chart program is abnormal, please contact the after-sales personnel",
    "zh_msg": "任务规划流图程序异常",
    "tip": "请联系售后人员",
    "ja_msg": "タスク プランニング フローチャート プログラムが異常です",
    "ja_tip": "アフターセールス担当者に連絡してください",
    "en_msg": "The task planning flow chart program is abnormal",
    "en_tip": "Please contact the after-sales personnel",
    "de_msg": "Unbekannter Fehler, undefinierter Fehler",
    "de_tip": "Bitte wenden Sie sich an das Kundendienstpersonal",
}

DEFAULT_ERROR = ALL_ERROR["99999"]

# 以下是为了兼容不同的字段名
for v in ALL_ERROR.values():
    v["code"] = v["error_code"]
    v["zh_tip"] = v["tip"]
    v["msg_type"] = ""
    v["class"] = "error"
