from xyz_motion import RobotDriver
from xyz_vision_flow.xyz_vision_bridge import XYZVisionBridge
from xyz_motion import SE3
def get_tf_base_end():
    input_step1 = None
    while input_step1 != "1":
        input_step1 = input("步骤1：放置一个sku，把机器人移动到准确的抓取位，然后切为自动模式，连上机器人节点。确认后按1+回车，强制退出按2+回车...\n")
        if input_step1 == "2":
            return None

    try:
        R = RobotDriver(0)
        tf_base_end = R.get_cartpose()
        print("cartpose为：\n")
        print(tf_base_end.xyz_quat)
        print("步骤1成功！\n")
    except:
        raise Exception("无法读取cartpose！请确认机器人已连接并处于自动模式！")
    return tf_base_end

def get_tf_base_tip():
    input_step2 = None
    while input_step2 != "1":
        input_step2 = input("步骤2：切换为手动模式移动机械手，不要触碰到箱子，使待抓取物暴露在相机视野内，然后切换回自动模式。确认后按1+回车，强制退出按2+回车...\n")
        if input_step2 == "2":
            return None
    try:
        R = RobotDriver(0)
        scan_pose = R.get_cartpose().xyz_quat
        print("scan_pose为：\n")
        print(scan_pose)
        print("步骤2成功！\n")
    except:
        raise Exception("无法获取scan pose！请确认机器人已连接并处于自动模式！")
    input_step3 = None
    while input_step3 != "1":
        input_step3 = input("步骤3：打开可以正确识别物体的xvf流图，并开启视觉服务。确认后按1+回车，强制退出按2+回车...\n")
        if input_step3 == "2":
            return None
    tote_id = None
    while not tote_id:
        tote_id = input("请输入用于拍照的tote_id，以回车结束：\n")
    print(f"tote_id确认为： {tote_id}")
    try:
        B = XYZVisionBridge()
        B.capture_images(tote_id)
        B.set_tote_scan_pose(tote_id, scan_pose)
        tf_base_tip = SE3(B.calculate_object_poses(tote_id)["results"][0]["pose"])
    except:
        raise Exception("无法获取视觉结果!")
    return tf_base_tip

tf_base_end = get_tf_base_end()
if not tf_base_end:
    print("强制退出成功！\n")
    quit()
tf_base_tip = get_tf_base_tip()
if not tf_base_tip:
    print("强制退出成功！\n")
    quit()
if tf_base_end and tf_base_tip:
    tf_end_tip = tf_base_end.inv() * tf_base_tip
    print("=====================自动标定tip完成========================\n")
    print("tip相对法兰的的位姿为：")
    print(tf_end_tip.xyz_quat)

