import yaml
import signal
import logging
import subprocess
import logging.config

logging_config = {
    'version': 1,
    'disable_existing_loggers': False,
    'formatters': {
        'colored': {
            '()': 'colorlog.ColoredFormatter',
            'format': "%(log_color)s[%(asctime)s %(levelname)7s][%(name)s]%(reset)s %(blue)s%(message)s%(reset)s%(log_color)s"
        }
    },
    'handlers': {
        'console': {
            'level': 'DEBUG',
            'formatter': 'colored',
            'class': 'logging.StreamHandler',
        },
    },
    'loggers': {
        'robot_node': {
            'handlers': ['console'],
            'level': logging.DEBUG,
            'propagate': True
        }
    }
}
logging.config.dictConfig(logging_config)
logger = logging.getLogger('robot_node')

process = None
start_command = "/opt/ros/noetic/bin/rosrun xyz_robot xyz_robot_node"

robot_config_path = '/home/xyz/xyz_app/app/robot_config/robot_config.yaml'
logger.info("正在加载robot_config相关配置...")
with open(robot_config_path) as f:
    config_data = yaml.load(f)
    robot_list = config_data['robot_list']
    num_robot = len(robot_list)
device_id = "2"
simulation = config_data["robot_list"][0]["simulation"]
robot_vender = config_data["robot_list"][0]["robot_type"]
support_plc_start_robot = ["abb", "kuka", "fanuc"]


def start():
    global start_command

    logger.info(f"正在启动机器人（{robot_vender}）......")
    # if not simulation:
        # if robot_vender in support_plc_start_robot:
        #     try:
        #         from xyz_io_client.io_client import is_auto_mode
        #         from xyz_io_client.io_client import is_enabled
        #         from xyz_io_client.io_client import is_in_pause
        #         from xyz_io_client.io_client import is_estop
        #         from xyz_io_client.io_client import is_error
        #         from xyz_io_client.io_client import is_collision
        #         from xyz_io_client.io_client import is_normal
        #         from xyz_io_client.io_client import start_robot

        #         # start robot
        #         logger.info("Starting robot...")
        #         start_robot(device_id)
        #         # start check
        #         logger.info("Checking is robot in pause")
        #         if is_in_pause(device_id) == True:
        #             error_message = "Robot in pause"
        #             logger.error(error_message)
        #             raise Exception(error_message)
                
        #         logger.info("Checking is robot in e-stop")
        #         if is_estop(device_id) == True:
        #             error_message = "Robot in e-stop"
        #             logger.error(error_message)
        #             raise Exception(error_message)
                
        #         logger.info("Checking is robot in error")
        #         if is_error(device_id) == True:
        #             error_message = "Robot in error"
        #             logger.error(error_message)
        #             raise Exception(error_message)
                
        #         logger.info("Checking is robot in collision")
        #         if is_collision(device_id) == True:
        #             error_message = "Robot in collision"
        #             logger.error(error_message)
        #             raise Exception(error_message)
                
        #         logger.info("Checking is robot in auto-mode")
        #         if is_auto_mode(device_id) == False:
        #             error_message = "Robot is not in auto mode"
        #             logger.error(error_message)
        #             raise Exception(error_message)
                
        #         logger.info("Checking is robot enabled")
        #         if is_enabled(device_id) == False:
        #             error_message = "Robot is not enabled"
        #             logger.error(error_message)
        #             raise Exception(error_message)

        #         logger.info("Checking is not normal")
        #         if is_normal(device_id) == False:
        #             error_message = "Robot is not normal"
        #             logger.error(error_message)
        #             raise Exception(error_message)
        #         # start robot
        #         start_robot(device_id)
        #     except Exception as e:
        #         logger.error(f"Start robot by plc failed, {repr(e)}", exc_info=True)

    # start xyz_robot_node
    proc = subprocess.Popen(args=start_command.split())
    logger.info(f"启动机器人节点，进程号{proc.pid}")
    return proc


def quit(signum, frame):
    global process
    logger.warning(f"正在关闭机器人节点，信号码{signum}...")
    process.send_signal(signum)

    # if not simulation and robot_vender in support_plc_start_robot:
    #     try:
    #         from xyz_io_client.io_client import stop_robot
    #         stop_robot(device_id)
    #     except Exception as e:
    #         logger.error("停止机器人服务时发生异常!", exc_info=True)


if __name__ == "__main__":
    process = start()

    signal.signal(signal.SIGTERM, quit)
    signal.signal(signal.SIGINT, quit)

    returncode = process.wait()
    logger.warning(f"程序退出, 响应码{returncode}")