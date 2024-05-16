import yaml
import os

robot_config_path = '/home/xyz/xyz_app/app/robot_config/robot_config.yaml'
with open(robot_config_path) as f:
    config_data = yaml.load(f)
    robot_list = config_data['robot_list']
    num_robot = len(robot_list)
device_id = "2"
simulation = config_data["robot_list"][0]["simulation"]
robot_vender = config_data["robot_list"][0]["robot_type"]
support_plc_start_robot = ["abb", "kuka", "fanuc"]


def start():
    print("Starting {} robot ......".format(robot_vender))
    if not simulation:
        if robot_vender in support_plc_start_robot:
            try:
                from xyz_io_client.io_client import is_auto_mode
                from xyz_io_client.io_client import is_enabled
                from xyz_io_client.io_client import is_in_pause
                from xyz_io_client.io_client import is_estop
                from xyz_io_client.io_client import is_error
                from xyz_io_client.io_client import is_collision
                from xyz_io_client.io_client import is_normal
                from xyz_io_client.io_client import start_robot

                # start robot
                print("Starting robot...")
                start_robot(device_id)
                # start check
                print("Checking is robot in pause")
                if is_in_pause(device_id):
                    raise Exception("Robot in pause")
                print("Checking is robot in e-stop")
                if is_estop(device_id):
                    raise Exception("Robot in e-stop")
                print("Checking is robot in error")
                if is_error(device_id):
                    raise Exception("Robot in error")
                print("Checking is robot in collision")
                if is_collision(device_id):
                    raise Exception("Robot in collision")
                print("Checking is robot in auto-mode")
                if not is_auto_mode(device_id):
                    raise Exception("Robot is not in auto mode")
                print("Checking is robot enabled")
                if not is_enabled(device_id):
                    raise Exception("Robot is not enabled")
                if not is_normal(device_id):
                    raise Exception("Robot is not normal")
            except Exception as error:
                print("start robot by plc failed {}".format(error))

    # start xyz_robot_node
    os.system("rosrun xyz_robot xyz_robot_node")
    # get xyz_robot_node pid.
    ''' can not get pid, process will block at xyz_robot_node process,
        if want to get, must start xyz_robot_node at thread
    pid = os.popen("ps -ef | grep xyz_robot_node").readlines()[0].split()[1]
    print(pid)
    '''

def stop(pid):
    # kill xyz_robot_node
    # os.kill(pid)
    # stop xyz_robot_node
    if robot_vender in support_plc_start_robot:
        try:
            from xyz_io_client.io_client import stop_robot
            stop_robot(device_id)
        except Exception as error:
            print("stop robot by plc failed {}".format(error))
    # kill this process
    # os.kill(os.getpid())


if __name__ == "__main__":
    pid = start()

    stop(pid)
