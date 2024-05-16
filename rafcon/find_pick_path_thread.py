
import copy,time,dill,math
from xyz_motion import CollisionChecker,SE3,RobotRos,PlanningEnvironmentRos
from xyz_env_manager.msg import AttachedCollisionObject,Pose
from queue import PriorityQueue,Empty
import multiprocessing
import queue
import threading
import concurrent.futures
import os
def sort_by_length(item):
   return len(item[1])  

def pose_to_list(container_item_origin):
   tf_map_object = []
   tf_map_object.append(container_item_origin.x)
   tf_map_object.append(container_item_origin.y)
   tf_map_object.append(container_item_origin.z)
   tf_map_object.append(container_item_origin.qx)
   tf_map_object.append(container_item_origin.qy)
   tf_map_object.append(container_item_origin.qz)
   tf_map_object.append(container_item_origin.qw)   
   return tf_map_object

def check_collision(our_robot,check_robot_list,planning_env,init_joints,container_item,stop_event):
   #获取当前抓取箱子坐标
   tf_map_object = SE3(pose_to_list(container_item.origin))
   tf_map_object = tf_map_object*SE3([0,0,-0.035,0,0,0,1])
   #获取抓取法兰所有姿态
   tf_tip_object_list = [[0,0,0,1,0,0,0],[0,0,0,0,1,0,0]]   
   tf_map_flange_list = []
   for tf_tip_object in tf_tip_object_list:
      tf_tip_object = SE3(tf_tip_object)
      tf_map_flange = tf_map_object * tf_tip_object.inv() * our_robot.get_active_tool().get_tf_endflange_tip("tip_0").inv()
      tf_map_flange_list.append(tf_map_flange.xyz_quat)  
   
   kinematic_solver = our_robot.get_kinematics_solver()

   #计算所有抓取姿态法兰joint
   check_joints = []
   init_joints_list = []
   init_joints = [0.0, -1.57, 1.4, 0.0]
   init_joints_list.append(init_joints)
   init_joints_list.append(init_joints[0:-1]+[init_joints[-1]+3.1415926])
   init_joints_list.append(init_joints[0:-1]+[init_joints[-1]-3.1415926])
   for i in tf_map_flange_list:
      if stop_event.is_set():
         return True
      for init_joint in init_joints_list:
         check_joint = kinematic_solver.compute_best_ik(SE3(i), init_joint)
         if not check_joint:
            continue
         else:
            if len(check_joint)==4:
               check_joint = kinematic_solver.convert_four_dof_to_six(check_joint)
               check_joints.append(check_joint) 
               break
            else:  
               check_joints.append(check_joint) 
               break 
   if not check_joints:                
      return True   
                

   #检测所有的姿态是否满足气缸碰撞条件，如果满足只取一组解
   collision_flag = True
   for joint_index,check_joint in enumerate(check_joints):
      for check_robot in check_robot_list:
         #添加碰撞检测器
         checker = CollisionChecker(check_robot, planning_env)   
         if checker.check_point_collision(check_joint):
            if container_item.additional_info.values[-3]=="9":
               pass
               #import ipdb;ipdb.set_trace()
            continue
         else:
            collision_flag = False   
            break      

   return collision_flag


#过滤得到顶层箱子
def filter_layer_items(items):

   combined_data = {}
   for item in items:
      #建立x,y坐标的键，同一列箱子xy坐标一致
      key = (round(item.origin.x,2), round(item.origin.y,2))
      if key not in combined_data.keys():
         combined_data[key] = item
      else:   
         # 只保留Z最大的类实例
         if item.origin.z > combined_data[key].origin.z:
               combined_data[key] = item

   new_items = list(combined_data.values())
   return new_items

#通过机器人限定搜索范围
#初始目标点干涉的范围
def check_first_robot_collisions(self, our_robot, check_robot_list, planning_env, init_joints, container_items,pick_items):
   pick_robot_collisions_dict = {}
   #得到目标点抓取箱子的干涉
   for pick_item in pick_items:
      tf_map_object = SE3(pose_to_list(container_item.origin))

      #获取抓取法兰所有姿态
      tf_tip_object_list = [[0,0,0,1,0,0,0],[0,0,0,0,1,0,0]]   
      tf_map_flange_list = []
      for tf_tip_object in tf_tip_object_list:
         tf_tip_object = SE3(tf_tip_object)
         tf_map_flange = tf_map_object * tf_tip_object.inv() * our_robot.get_active_tool().get_tf_endflange_tip("tip_0").inv()
         tf_map_flange_list.append(tf_map_flange.xyz_quat)   
      
      kinematic_solver = our_robot.get_kinematics_solver()

      #计算所有抓取姿态法兰joint
      check_joints = []
      #有可能ik无解，所以旋转法兰轴180
      init_joints_list = []
      init_joints_list.append(init_joints)
      init_joints_list.append(init_joints[0:-1]+[init_joints[-1]+3.1415926])
      init_joints_list.append(init_joints[0:-1]+[init_joints[-1]-3.1415926])
      for i in tf_map_flange_list:
         for init_joint in init_joints_list:
            check_joint = kinematic_solver.compute_best_ik(SE3(i), init_joint)
            if not check_joint:
               check_joint = kinematic_solver.compute_best_ik(SE3(i), init_joint)
            else:
               if len(check_joint)==4:
                  check_joint = kinematic_solver.convert_four_dof_to_six(check_joint)
                  check_joints.append(check_joint) 
                  break
               else:  
                  check_joints.append(check_joint) 
                  break 
      if not check_joints:              
         continue      
      #应该分别计算不同抓取姿态下的碰撞箱子，来回切换会导致干涉箱子很少   
      robot_collision_items_dict = {}            
      for joints in check_joints:
         robot_collision_items = [] 
         #遍历哪些箱子干涉       
         for container_item in container_items:
            #清空环境，并添加箱子
            planning_env.clear_container_all_items("0")
            planning_env.add_container_items("0",[container_item])

            #检测所有的姿态是否满足气缸碰撞条件，如果满足只取一组解
            collision_flag = True
            for check_robot in check_robot_list:
               #添加碰撞检测器
               checker = CollisionChecker(check_robot, planning_env)   
               if checker.check_point_collision(joints):
                  continue
               else:
                  collision_flag = False   
                  break    
            if collision_flag:
               robot_collision_items.append(container_item)
         robot_collision_items_dict[tuple(joints)] = robot_collision_items
      #取两种抓取姿态最少的碰撞箱子
      robot_collision_items = sorted(robot_collision_items_dict.items(),key=sort_by_length)[0][1]     
      #整合数据，只取顶层箱子当后续索引值
      pick_robot_collisions_dict[pick_item.additional_info.values[-3]] = filter_layer_items(robot_collision_items)
   return pick_robot_collisions_dict
   
#通过初始机器人目标点的干涉箱子，对需要搜索的箱子进行排序，尽可能迅速计算得到目标点
def sorted_robot_collisions(remaining_items, pick_robot_collisions):
   new_items_0 = []
   new_items_1 = []
   for item in remaining_items:
      xy_check_item = list(filter(lambda x:x.origin.x==item.origin.x and x.origin.y==item.origin.y,pick_robot_collisions)) 
      if xy_check_item:
         new_items_0+=[item]
      else:
         new_items_1+=[item]
   return new_items_0 + new_items_1 


class Node:
   def __init__(self, value):
      self.value = value
      self.next = None

class Stack:
   def __init__(self):
      self.top = None

   def push(self, value):
      new_node = Node(value)
      new_node.next = self.top
      self.top = new_node

   def pop(self):
      if self.top is None:
         return None
      popped_value = self.top.value
      self.top = self.top.next
      return popped_value

   def is_empty(self):
      return self.top is None
       

#TODO 深度优先DFS算法，缺少搜索缓存深度学习,搜索范围优先  
class DFS_ASYNC:

   def __init__(self) -> None:

      # 初始化一个字典用于存储已访问的状态
      self.visited_states = {}
      # 进程计算队列
      self.calculation_queue = None
      # 进程终止事件
      self.termination_event = None
      # 日志
      self.logger = None

      #内部进程队列
      self.my_queue = multiprocessing.Queue()
      #内部进程终止事件
      self.event = multiprocessing.Event()

      self.time_out = 60

   def check_all_collisions(self,our_robot, check_robot_list, planning_env, 
                            init_joints, next_item,stack,state_key,path,min_path_length,depth,stop_event,open_set_lock):

      try:
         next_item_id = next_item.additional_info.values[-3]
         new_state_key = state_key + [next_item.name]
         #判断箱子是否干涉
         if not check_collision(our_robot, check_robot_list,planning_env, init_joints, next_item,stop_event):
            #不干涉条件满足，添加到路径里
            new_path = path + [next_item_id]
            #如果还没有目标点路径，继续添加栈数据
            if not min_path_length:
               with open_set_lock:
                  stack.push((next_item,new_state_key,new_path,depth + 1)) 
               
            #已有目标点路径
            else:
               #当前路径小于最小路径
               if len(new_path)<min_path_length:
                  with open_set_lock:
                     stack.push((next_item,new_state_key,new_path,depth + 1))                    
      except:
            if stop_event.is_set():
               self.logger(f"stop_event")   
            else:   
               self.logger(f"error")    
            return False                  


   def stack_search(self,our_robot, check_robot_list,planning_env,init_joints,container_items,init_node,pick_items,direction):

      # 初始化队列
      stack = Stack() 

      # 初始化所有路径列表
      all_paths = [] 

      #最小路径，这个也是为了不算所有解
      min_path_length = None
      # copy了一份初始解的环境，便于后续环境里箱子的操作
      init_planning_env = copy.copy(planning_env)

      #纸箱 box id
      box_id = init_node.additional_info.values[-3]
      #定义初始栈数据，添加一个搜索参数
      stack.push((init_node, [init_node.name],[box_id],0))

      # 创建一个锁
      open_set_lock = threading.Lock()
      # 创建一个停止事件
      stop_event = threading.Event()
      #进入栈的循环,栈不为空就一直循环

      with concurrent.futures.ThreadPoolExecutor(max_workers=100) as executor:
         # 进入优先队列的循环,队列不为空就一直循环   
         while not stack.is_empty() and not self.termination_event.is_set() and not self.event.is_set():
            try:            
               #调出栈数据
               current_node, state_key, path,depth= stack.pop()
               
               #如果搜索所有最短路径，则在当前路径下+1后与最短路径比较
               check_path = len(path)+1  
               
               #如果箱子在目标箱子里，则添加到所有路径里
               if current_node in pick_items:
                  #搜索结束，添加路径，并定义更新最小路径长度
                  all_paths.append(path)
                  min_path_length = len(path)
                  continue 
               #如果不在目标箱子里，则判断当前数据的路径长度
               else:
                  if min_path_length and check_path>=min_path_length:
                     continue 

               #将路径节点ID当成键值
               current_state_key = tuple(sorted(state_key))

               # 判断是否已经访问过这个状态
               if current_state_key in self.visited_states:          
                  continue   



               # 初始化环境
               planning_env = copy.copy(init_planning_env)

               # 清除当前路径下箱子的环境
               planning_env.clear_container_items("0", [name for name in state_key])

               # 过滤掉已搜索路径的箱子
               remaining_items = list(filter(lambda x: x.name not in [name for name in state_key], container_items))
               # 过滤非顶层箱子
               remaining_items = filter_layer_items(remaining_items)


               # 搜索剩余可抓箱子是否与环境干涉
               for next_item in remaining_items:

                  executor.submit(self.check_all_collisions, our_robot, check_robot_list, planning_env, init_joints, next_item,stack,state_key,path,min_path_length,depth,stop_event,open_set_lock)
                  
            except:
               queue.Empty
               self.logger(f"queue empty")
               break   
         if all_paths:   
            all_paths = list(filter(lambda x:len(x)==len(min(all_paths,key=lambda x: len(x))),all_paths))
            self.logger(f"{direction}面,起始点{box_id}的最短路径为{all_paths}")  
         else:
            self.logger(f"{direction}面,起始点{box_id}没有找到路径")
         self.my_queue.put(all_paths)   
         stop_event.set()
         # 关闭线程池，不再接受新任务           
         executor.shutdown(wait=False) 
         return all_paths         
            

   def find_all_paths(self, out_robot_msg, check_robot_list_msg, planning_env_msg, init_joints, container_items_msg, filtered_items, pick_items,direction):
      """
      深度优先DFS算法
      Parameters:
      - out_robot_msg: 初始的robot,用于得到tip、tool等数据
      - check_robot_list_msg: 一个用于计算碰撞的robot列表
      - planning_env_msg: 环境，用于反复操作
      - init_joints: 机器人初始点位，用于求解抓取点
      - container_items_msg: 当前环境的箱子
      - filtered_items_msg: 一开始可以抓的解，也就是起始点
      - pick_items_msg:目标箱子
      - direction:托盘的哪个面
      - termination_event:终止循环事件
      - calculation_queue:通信计算队列

      Returns:
      返回一个列表，列表按抓取顺序排列
      """


      # 获取当前进程的 ID
      current_process_id = os.getpid()

      our_robot = RobotRos.from_ros_msg(dill.loads(out_robot_msg))
      check_robot_list_msg = dill.loads(check_robot_list_msg)
      planning_env = PlanningEnvironmentRos.from_ros_msg(dill.loads(planning_env_msg))
      check_robot_list = []
      for i in check_robot_list_msg:
         check_robot_list.append(RobotRos.from_ros_msg(i))   
      container_items = dill.loads(container_items_msg)
      filtered_items = dill.loads(filtered_items)
      pick_items = dill.loads(pick_items)


      self.logger(f"{current_process_id}进程,{direction}面开始计算,进程终止事件状态{self.termination_event.is_set()}")

      stack = Stack()
      #初始化所有路径列表
      all_paths = []
      # 创建一个停止事件
      stop_event = threading.Event()
      
      #copy了一份初始解的环境，便于后续环境里箱子的操作
      init_planning_env = copy.copy(planning_env)      
      
      #最小路径，这个也是为了不算所有解
      min_path_length = None

      #filtered_items 是一开始可以抓的解，从这些解遍历

      for start_point in filtered_items:

         #纸箱 box id
         box_id = start_point.additional_info.values[-3]
         #定义初始栈数据，添加一个搜索参数
         stack.push((start_point, [start_point.name],[box_id],0))


      #进入栈的循环,栈不为空就一直循环
      current_time = time.time()
      while not stack.is_empty():
         if time.time() - current_time > self.time_out:
            self.logger("已超过计算时间")
            break
         #调出栈数据
         current_node, state_key, path,depth= stack.pop()
         
         #如果搜索所有最短路径，则在当前路径下+1后与最短路径比较
         check_path = len(path)+1    
   
         #如果箱子在目标箱子里，则添加到所有路径里
         if current_node in pick_items:
            #搜索结束，添加路径，并定义更新最小路径长度
            all_paths.append(path)
            min_path_length = len(path)
            self.logger(f"已找到一个解，路径为{path}")
            continue 
         #如果不在目标箱子里，则判断当前数据的路径长度
         else:
            if min_path_length and check_path>=min_path_length:
               continue 

         #将路径节点ID当成键值
         current_state_key = tuple(sorted(state_key))

         # 判断是否已经访问过这个状态
         if current_state_key in self.visited_states:          
            continue   

         # 将当前状态添加到已访问的字典中
         self.visited_states[current_state_key] = current_node.name
               

         #初始化环境   
         planning_env = copy.copy(init_planning_env)

         #清除当前路径下箱子的环境
         planning_env.clear_container_items("0", [name for name in state_key]) 
         
         #过滤掉已搜索路径的箱子
         remaining_items = list(filter(lambda x:x.name not in [name for name in state_key],container_items))
         
         #过滤非顶层箱子
         remaining_items = filter_layer_items(remaining_items)
         
         #搜索剩余可抓箱子是否与环境干涉
         for next_item in remaining_items:
            next_item_id = next_item.additional_info.values[-3]
            new_state_key = state_key + [next_item.name]
            #判断箱子是否干涉
            if not check_collision(our_robot, check_robot_list,planning_env, init_joints, next_item,stop_event):

               #不干涉条件满足，添加到路径里
               new_path = path + [next_item_id]

               #如果还没有目标点路径，继续添加栈数据
               if not min_path_length:
                  stack.push((next_item,new_state_key,new_path,depth + 1)) 
                  
               #已有目标点路径
               else:
                  #当前路径小于最小路径
                  if len(new_path)<min_path_length:
                     stack.push((next_item,new_state_key,new_path,depth + 1))   
                  else:
                     break                               

      return all_paths




class CostItem:

   def __init__(self, f_cost,g_cost,current_node,state_key,parent=None):
      self.f_cost = f_cost
      self.g_cost = g_cost
      self.current_node = current_node
      self.state_key = state_key 
      self.parent = parent
   def __lt__(self, other):
      return self.f_cost < other.f_cost

   # 从当前节点逆向回溯以重建路径
   def reconstruct_path(self):
      path = []
      current = self
      while current:
         path.insert(0, current.current_node.additional_info.values[-3])
         current = current.parent
      return path
   
#TODO A*算法，启发式函数需要优化 
"""

A*算法,总代价=实际代价+预估的启发式代价，从而优先搜索具有更低总代价的节点

""" 

class A_STAR_ASYNC():

   def __init__(self) -> None:

      #缓存来存储已计算的启发式代价
      self.heuristic_cache = {}

      # 初始化一个字典用于存储已访问的状态
      self.visited_states = {}
      
      # 进程计算队列
      self.calculation_queue = None
      # 进程终止事件
      self.termination_event = None
      # 日志
      self.logger = None


   def next_overall_cost(self,next_item, finished_items,scale):
      overall_cost = 0
      for finished_item in finished_items:
         x_cost = abs(next_item.origin.x - finished_item.origin.x)
         y_cost = abs(next_item.origin.y - finished_item.origin.y)
         overall_cost = overall_cost+x_cost+y_cost
      return overall_cost/scale  
   
   #用于实际代价
   def g_current_cost(self,start_node, target_node):
      """
      当前代价代价函数

      Parameters:
      - start_node: 起始点
      - target_node: 目标点

      Returns:
      返回一个代价值cost
      """  
      dx = abs(start_node.origin.x - target_node.origin.x)
      dy = abs(start_node.origin.y - target_node.origin.y)
      dz = abs(start_node.origin.z - target_node.origin.z)

      return dx + dy + dz

   #需要根据垛型去计算当前节点到预估点的代价，目前暂时不太好一种算法计算不同垛型
   def heuristic_cost_estimate(self,start_node, target_node):
      """
      启发式代价函数

      Parameters:
      - start_node: 起始点
      - target_node: 目标点

      Returns:
      返回一个代价值cost
      """   
      start_node_id = int(start_node.additional_info.values[-3])
      target_node_id = int(target_node.additional_info.values[-3])
      #通过哈希表记录每次代价计算如果存在已计算过的，直接获取
      if (start_node_id, target_node_id) in self.heuristic_cache:
         return self.heuristic_cache[(start_node_id, target_node_id)]  

      dx = abs(start_node.origin.x - target_node.origin.x)
      dy = abs(start_node.origin.y - target_node.origin.y)
      dz = abs(start_node.origin.z - target_node.origin.z)
      
      #对角线距离
      # 在水平、垂直和垂直方向上的移动成本
      diagonal_cost = min(dx, dy, dz)

      # 剩余移动成本
      remaining_cost = abs(dx - dy) + abs(dy - dz) + abs(dz - dx)

      # 计算总成本
      #对角线距离
      total_cost = int(diagonal_cost * math.sqrt(3) + remaining_cost)
          
      # #曼哈顿距离
      total_cost = dx+dy+dz        
      # 将计算结果放入缓存
      self.heuristic_cache[(start_node_id, target_node_id)] = total_cost

      return total_cost
   
   def open_set_search(self,our_robot, check_robot_list,planning_env,init_joints,container_items,init_node,pick_items):

      # 初始化优先队列
      open_set = PriorityQueue() 

      # 初始化所有路径列表
      all_paths = [] 

      # 初始化标志变量
      found_shortest_path = False

      # copy了一份初始解的环境，便于后续环境里箱子的操作
      init_planning_env = copy.copy(planning_env)

      # 将f(n)代价，当前代价g(n)，当前节点，路径ID,环境，箱子放入队列
      for target_item in pick_items:
         h_cost = self.heuristic_cost_estimate(init_node,target_item)
         f_cost = h_cost+0
         init_node_id = init_node.additional_info.values[-3]
         cost_item = CostItem(f_cost,0,init_node,[init_node.name])    
         open_set.put(cost_item)  

      # 进入优先队列的循环,队列不为空就一直循环   
      while not open_set.empty() and not found_shortest_path and not self.termination_event.is_set() and not self.event.is_set():
         # 取出所有的栈数据
         retrieved_item = open_set.get()
         f_cost = retrieved_item.f_cost
         g_cost = retrieved_item.g_cost
         current_node = retrieved_item.current_node
         state_key = retrieved_item.state_key
         # 如果箱子在目标箱子里，则添加到所有路径里
         if current_node in pick_items:
            # 搜索结束，回溯路径
            all_paths.append(retrieved_item.reconstruct_path())
            # 设置标志变量，告知找到最短路径
            found_shortest_path = True
            break


         #将路径节点ID当成键值
         current_state_key = tuple(sorted(state_key))
         # 判断是否已经访问过这个状态
         if current_state_key in self.visited_states:          
            continue   

         # 将当前状态添加到已访问的字典中
         self.visited_states[current_state_key] = current_node.name

         # 初始化环境
         planning_env = copy.copy(init_planning_env)

         # 清除当前路径下箱子的环境
         planning_env.clear_container_items("0", [name for name in state_key])

         # 过滤掉已搜索路径的箱子
         remaining_items = list(filter(lambda x: x.name not in [name for name in state_key], container_items))
         finished_items = list(filter(lambda x: x.name in [name for name in state_key], container_items))
         # 过滤非顶层箱子
         remaining_items = filter_layer_items(remaining_items)

         # 搜索剩余可抓箱子是否与环境干涉
         for next_item in remaining_items:
            # 抓取箱子在已抓取箱子的整体权重,也就是XY的偏置
            overall_cost = self.next_overall_cost(next_item,finished_items,1)
            #节点ID
            next_item_id = next_item.additional_info.values[-3]

            # 判断箱子是否干涉
            if not check_collision(our_robot, check_robot_list, planning_env, init_joints, next_item):
               # 不干涉条件满足，节点ID到路径里
               new_state_key = state_key + [next_item.name]

               # 计算总代价
               for target_item in pick_items:

                  #计算预估代价
                  h_cost = self.heuristic_cost_estimate(next_item, target_item)
                  #更新当前g(n)代价
                  g_cost = self.g_current_cost(current_node,next_item)+g_cost
                  #更新将f(n)代价
                  f_cost = g_cost + h_cost + overall_cost

                  # 将f(n)代价，当前代价g(n)，当前节点，路径ID,环境,箱子放入队列
                  cost_item = CostItem(f_cost,g_cost,next_item,new_state_key,retrieved_item)  
                  open_set.put(cost_item)    

      self.my_queue.put(all_paths) 
      return all_paths              


   def check_all_collisions(self,our_robot, check_robot_list, planning_env, 
                            init_joints, next_item,overall_cost,open_set,state_key,pick_items,current_node,retrieved_item,g_cost,open_set_lock,stop_event):
      try:
         #节点ID到路径里
         new_state_key = state_key + [next_item.name]
         # 判断箱子是否干涉
         if not check_collision(our_robot, check_robot_list, planning_env, init_joints, next_item,stop_event):
            # 计算总代价
            for target_item in pick_items:

               #计算预估代价
               h_cost = self.heuristic_cost_estimate(next_item, target_item)
                          
               #更新当前g(n)代价
               g_cost = self.g_current_cost(current_node,next_item)+g_cost
               #更新将f(n)代价
               f_cost = g_cost + h_cost + overall_cost

               # 将f(n)代价，当前代价g(n)，当前节点，路径ID,环境,箱子放入队列
               cost_item = CostItem(f_cost,g_cost,next_item,new_state_key,retrieved_item)  
               with open_set_lock:
                  open_set.put(cost_item) 
         return True 
      except Exception as e:
            self.logger(f"check_all_collisions error:{str(e)}")
            if stop_event.is_set():
               self.logger(f"stop_event")   
            else:   
               self.logger(f"error")    
            return False    
   
   def path_check(self,current_process_id,path,container_items,our_robot,check_robot_list,planning_env,init_joints,stop_event):
      current_process_id
      #偏置
      slide_list = [[0.08, 0.08, 0.07, 0, 0, 0, 1], [0.08, -0.08, 0.07, 0, 0, 0, 1], [-0.08, 0.08, 0.07, 0, 0, 0, 1], [-0.08, -0.08, 0.07, 0, 0, 0, 1]]
      # 遍历路径
      self.logger(f"进程{current_process_id},开始后验路径{path}")
      for node in path:
         self.logger(f"进程{current_process_id},后验节点{node}")
         #获取当前节点箱子
         pick_item = list(filter(lambda x: x.additional_info.values[-3] == node, container_items))
         if len(pick_item)!=1:
            self.logger(f"进程{current_process_id},后验检查,{node}不在容器中")
            return False        
         pick_item = pick_item[0]
         planning_env.clear_container_items("0", [pick_item.name])
         attached_collision_object = AttachedCollisionObject()
         attached_collision_object.reference_tip_name = 'tip_0'
         attached_collision_object.reference_object_name = pick_item.name
         attached_collision_object.objects = [pick_item]
         attached_collision_object.from_workspace_id = "0"
         attached_collision_object.to_workspace_id = "1"
         attached_collision_object.origin_tip_transform = Pose(0,0,0,1,0,0,0)
         our_robot.detach_object()
         our_robot.attach_object(attached_collision_object,[])
         for check_robot in check_robot_list:
            check_robot.detach_object()  
            check_robot.attach_object(attached_collision_object,[])          
         slide_flag = False
         #遍历偏置
         for slide in slide_list:
            #更新偏置
            check_item = copy.deepcopy(pick_item)
            check_item.origin.x = check_item.origin.x+slide[0]
            check_item.origin.y = check_item.origin.y+slide[1]
            check_item.origin.z = check_item.origin.z+slide[2]
            if not check_collision(our_robot,check_robot_list,planning_env,init_joints,check_item,stop_event):
               slide_flag = True
               break     
            else:
               continue 
         if slide_flag:
            self.logger(f"进程{current_process_id},{node}验证ok,偏置{slide}无碰撞,继续搜索")
            continue
         else:
            self.logger(f"进程{current_process_id},{node}验证失败")
            our_robot.detach_object()   
            for check_robot in check_robot_list:
               check_robot.detach_object()              
            return False 
         
      our_robot.detach_object()   
      for check_robot in check_robot_list:
         check_robot.detach_object()                    
      return True
                                         

   def async_a_star_search(self,out_robot_msg, check_robot_list_msg, planning_env_msg, init_joints,\
                           container_items_msg,filtered_items_msg,pick_items_msg,direction,current_direction):
      """
      A* 路径搜索算法:f(n) = g(n) + h(n)
      Parameters:
      - out_robot_msg: 初始的robot,用于得到tip、tool等数据
      - check_robot_list_msg: 一个用于计算碰撞的robot列表
      - planning_env_msg: 环境，用于反复操作
      - init_joints: 机器人初始点位，用于求解抓取点
      - container_items_msg: 当前环境的箱子
      - filtered_items_msg: 一开始可以抓的解，也就是起始点
      - pick_items_msg:目标箱子
      - direction:托盘的哪个面
      - termination_event:终止循环事件
      - calculation_queue:通信计算队列

      Returns:
      返回一个列表，列表按抓取顺序排列
      """
      # 获取当前进程的 ID
      current_process_id = os.getpid()
      try:
         self.logger(f"{current_process_id}进程,{direction}面开始计算,进程终止事件状态{self.termination_event.is_set()}")
         planning_env = PlanningEnvironmentRos.from_ros_msg(dill.loads(planning_env_msg))  
         our_robot = RobotRos.from_ros_msg(dill.loads(out_robot_msg))
         check_robot_list_msg = dill.loads(check_robot_list_msg)
         check_robot_list = []
         for i in check_robot_list_msg:
            check_robot_list.append(RobotRos.from_ros_msg(i))   
         container_items = dill.loads(container_items_msg)
         pick_items = dill.loads(pick_items_msg)
         filtered_items = dill.loads(filtered_items_msg)
         
         self.logger(f"{current_process_id}进程,计算初始可抓取箱子为{[i.additional_info.values[-3] for i in filtered_items]}")
         # 初始化优先队列
         open_set = PriorityQueue() 

         # 初始化所有路径列表
         all_paths = [] 

         # 初始化标志变量
         found_shortest_path = False
         # 拷贝初始环境
         init_planning_env = copy.copy(planning_env)
         
         # 预先计算所有可能的起始点到目标点的代价
         for init_node in filtered_items:
            for target_item in pick_items:
               h_cost = self.heuristic_cost_estimate(init_node,target_item)
               f_cost = h_cost+0
               init_node_id = init_node.additional_info.values[-3]
               cost_item = CostItem(f_cost,0,init_node,[init_node.name])    
               open_set.put(cost_item)  


         # 创建一个锁
         open_set_lock = threading.Lock()
         # 创建一个停止事件
         stop_event = threading.Event()


         with concurrent.futures.ThreadPoolExecutor() as executor:
            # 进入优先队列的循环,队列不为空就一直循环   
            calculation_current_time = time.time()
            calculation_max_time = 180
            
            while not found_shortest_path and not self.termination_event.is_set():
               if time.time()-calculation_current_time > calculation_max_time:
                  self.logger(f"{current_process_id}进程,{direction}面计算超时")
                  break
                  
               try:            
                  # 取出所有的栈数据
                  retrieved_item = open_set.get(timeout=5)
                  f_cost = retrieved_item.f_cost
                  g_cost = retrieved_item.g_cost
                  current_node = retrieved_item.current_node
                  state_key = retrieved_item.state_key
                  
                  # 如果箱子在目标箱子里，则添加到所有路径里
                  if current_node in pick_items:
                     self.logger(f"{current_process_id}进程,优先队列剩余数量{open_set.qsize()}")
                     #TODO 添加回溯判断，避免后续运动规划无解 
                     # 搜索结束，回溯路径
                     all_paths.append(retrieved_item.reconstruct_path())
                     # 设置标志变量，告知找到最短路径
                     # found_shortest_path = True
                     # stop_event.set()
                     # break                     
                     # 初始化环境
                     planning_env = copy.copy(init_planning_env)
                     if self.path_check(current_process_id,all_paths[0],container_items,our_robot,check_robot_list,planning_env,init_joints,stop_event):
                        # 设置标志变量，告知找到最短路径
                        found_shortest_path = True
                        stop_event.set()
                        break
                     else:
                        self.logger(f"{current_process_id}进程,已搜索到路径,但是路径不可行,重新搜索")
                        all_paths = []
                        continue

                  #TODO 重复访问过滤会导致后验失效，先关闭            
                  #将路径节点ID当成键值
                  current_state_key = tuple(sorted(state_key))
                  # 判断是否已经访问过这个状态
                  if current_state_key in self.visited_states:          
                     continue   

                  # 将当前状态添加到已访问的字典中
                  self.visited_states[current_state_key] = current_node.name

                  # 初始化环境
                  planning_env = copy.copy(init_planning_env)

                  # 清除当前路径下箱子的环境
                  planning_env.clear_container_items("0", [name for name in state_key])

                  # 过滤掉已搜索路径的箱子
                  remaining_items = list(filter(lambda x: x.name not in [name for name in state_key], container_items))
                  finished_items = list(filter(lambda x: x.name in [name for name in state_key], container_items))
                  
                  # 过滤非顶层箱子
                  remaining_items = filter_layer_items(remaining_items)
                  
                  # 搜索剩余可抓箱子是否与环境干涉
                  for next_item in remaining_items:
                     # 抓取箱子在已抓取箱子的整体权重,也就是XY的偏置
                     overall_cost = self.next_overall_cost(next_item,finished_items,1)
                     executor.submit(self.check_all_collisions, our_robot, check_robot_list, planning_env, 
                     init_joints, next_item,overall_cost,open_set,state_key,pick_items,current_node,retrieved_item,g_cost,open_set_lock,stop_event)
               except Empty:
                  self.logger(f"{current_process_id}进程,队列已为空,但未搜索到路径")
                  break    
               except Exception as e:
                  self.logger(f"{current_process_id}进程,异常信息{str(e)}")
                  break  
         # 关闭线程池，不再接受新任务
         executor.shutdown(wait=True)   
            #open_set.task_done()  
         calculation_dict = {}
         if not all_paths:
            self.logger(f"{current_process_id}进程,{direction}面计算终止,进程终止事件状态{self.termination_event.is_set()}")
            return calculation_dict
         else:           
            if current_direction!=direction:
               self.logger(f"{current_process_id}进程,{direction}面计算得到{all_paths},但是转向,等待中")
               wait_time = 60
               current_time = time.time()
               while True:
                  if time.time()-current_time>wait_time or self.termination_event.is_set():
                     break
            self.logger(f"{current_process_id}进程,{direction}面计算得到{all_paths},进程终止事件状态{self.termination_event.is_set()}")
            calculation_dict["all_paths"] = all_paths
            calculation_dict["direction"] = direction     
            self.calculation_queue.put(calculation_dict)    
            return calculation_dict
      except Exception as e:
         self.logger(f"{current_process_id}进程,{direction}面计算异常,异常信息{str(e)}")