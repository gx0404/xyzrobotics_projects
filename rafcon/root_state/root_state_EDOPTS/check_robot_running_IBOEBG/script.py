
from xyz_motion import RobotDriver
import threading
import time



# 这是一个包装器函数，用于在单独的线程中执行阻塞调用
def thread_function(pin, result_container):
    r = RobotDriver(0)
    result = r.get_digital_input(pin)
    result_container.append(result)

def execute_with_timeout(pin, timeout_seconds):
    result_container = []  # 用于存储线程中的结果
    thread = threading.Thread(target=thread_function, args=(pin, result_container))
    
    # 启动线程
    thread.start()
    
    # 等待线程完成或超时
    thread.join(timeout_seconds)
    
    # 检查线程是否仍然活跃（意味着已经超时）
    if thread.is_alive():
        return None
    else:
        return result_container[0]

def execute(self, inputs, outputs, gvm):
    self.logger.info("Hello {}".format(self.name))
    result = execute_with_timeout(17, 0.5) 
    if result:
        return "success"
    else:
        self.logger.info("获取机器人超时,机器人未运行")
        time.sleep(5)
        return "hold"    


