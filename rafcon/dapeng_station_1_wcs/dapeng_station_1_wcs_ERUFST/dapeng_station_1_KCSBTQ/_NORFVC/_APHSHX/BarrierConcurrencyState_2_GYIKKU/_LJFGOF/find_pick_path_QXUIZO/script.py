import sys
sys.path.append('/home/xyz/xyz_app/app/rafcon/')

from xyz_motion import PlanningEnvironmentRos
from xyz_env_manager.client import get_planning_environment
from xyz_motion import AllRobotToolStatesRos,SE3,ToolRos,RobotRos#,pose_to_list
from xyz_env_manager.client import get_all_robot_tool_states
from xyz_motion import RobotDriver
import copy,time,dill,math
from xyz_motion import CollisionChecker
from queue import PriorityQueue
import numpy as np
import tf.transformations as tfm
from xyz_env_manager.msg import AttachedCollisionObject,Pose
import multiprocessing
import queue
import threading
import concurrent.futures

from find_pick_path_thread import A_STAR_ASYNC,DFS_ASYNC
from rafcon.xyz_exception_base import XYZExceptionBase

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

def check_collision(our_robot,check_robot_list,planning_env,init_joints,container_item,show_flag=None):
   #获取当前抓取箱子坐标
   tf_map_object = SE3(pose_to_list(container_item.origin))
   global check_collision_num 
   check_collision_num += 1
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
   init_jonints_list = []
   init_joints[-1] = 0
   init_jonints_list.append(init_joints)
   init_jonints_list.append(init_joints[0:-1]+[init_joints[-1]+3.1415926])
   init_jonints_list.append(init_joints[0:-1]+[init_joints[-1]-3.1415926])
   for i in tf_map_flange_list:
      for init_joint in init_jonints_list:
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
      return True   
                

   #检测所有的姿态是否满足气缸碰撞条件，如果满足只取一组解
   collision_flag = True
   for joint_index,check_joint in enumerate(check_joints):
      if not collision_flag:
         break
      for check_robot in check_robot_list:
         #添加碰撞检测器
         checker = CollisionChecker(check_robot, planning_env)   
         if checker.check_point_collision(check_joint):
            if show_flag:
               import ipdb;ipdb.set_trace()
            if container_item.additional_info.values[-3]=="79":
               pass
               #import ipdb;ipdb.set_trace()
            continue
         else:
            if show_flag:
               import ipdb;ipdb.set_trace()            
            if container_item.additional_info.values[-3]=="78":
               pass
               #import ipdb;ipdb.set_trace()
            collision_flag = False   
            break  
         
   #如果抓取无碰撞,计算下放置多的向下偏置       
   if not collision_flag:
      #考虑到拣配还原放置需要坐标偏移，因此需要调整
      tf_map_object = tf_map_object*SE3([0,0,-0.035,0,0,0,1])               
      #获取抓取法兰所有姿态
      tf_tip_object_list = [[0,0,0,1,0,0,0],[0,0,0,0,1,0,0]]   
      tf_map_flange_list = []
      for tf_tip_object in tf_tip_object_list:
         tf_tip_object = SE3(tf_tip_object)
         tf_map_flange = tf_map_object * tf_tip_object.inv() * our_robot.get_active_tool().get_tf_endflange_tip("tip_0").inv()
         tf_map_flange_list.append(tf_map_flange.xyz_quat)   
      

      #计算所有抓取姿态法兰joint
      check_joints = []
      for i in tf_map_flange_list:
         for init_joint in init_jonints_list:
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
         return True   
                  
      #检测所有的姿态是否满足气缸碰撞条件，如果满足只取一组解
      collision_flag = True
      for joint_index,check_joint in enumerate(check_joints):
         if not collision_flag:
            break
         for check_robot in check_robot_list:
            #添加碰撞检测器
            checker = CollisionChecker(check_robot, planning_env)   
            if checker.check_point_collision(check_joint):
               if show_flag:
                  import ipdb;ipdb.set_trace()
               if container_item.additional_info.values[-3]=="79":
                  pass
                  #import ipdb;ipdb.set_trace()
               continue
            else:
               if show_flag:
                  import ipdb;ipdb.set_trace()            
               if container_item.additional_info.values[-3]=="78":
                  pass
               collision_flag = False   
               break  
                   
   return collision_flag




#过滤得到顶层箱子
def filter_layer_items(items,row_flag=True):
   #两种模式,一种只取最高层，另一种则是每列箱子的最高层
   if row_flag:
      combined_data = {}
      for item in items:
         #建立x,y坐标的键，同一列箱子xy坐标一致
         key = (round(item.origin.x,2), round(item.origin.y,2))
         if key not in combined_data.keys():
            #判断原先字典是否有xy近似的key的标志flag
            check_key_flag = False
            for check_key in combined_data.keys():
               #判断绝对值是否小于0.015，如果xy都小于0.015，则认为是同列箱子
               if abs(item.origin.x-check_key[0])<0.015 and abs(item.origin.y-check_key[1])<0.015:    
                  check_key_flag = True
                  break      
                #如果不存在标志,则说明是个新列                   
            if not check_key_flag:                    
               combined_data[key] = item
            #如果存在,则说明是老列,则需要判断是否保留z最大的实例   
            else:
               if item.origin.z > combined_data[check_key].origin.z:
                  combined_data[check_key] = item        
         else:   
            # 只保留Z最大的类实例
            if item.origin.z > combined_data[key].origin.z:
               combined_data[key] = item

      new_items = list(combined_data.values())
        
   #只考虑最高列,不考虑每列层数不同   
   else:
      max_z = max(i.origin.z for i in items)
      new_items = list(filter(lambda x:abs(x.origin.z-max_z)<0.1,items))
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
      init_jonints_list = []
      init_jonints_list.append(init_joints)
      init_jonints_list.append(init_joints[0:-1]+[init_joints[-1]+3.1415926])
      init_jonints_list.append(init_joints[0:-1]+[init_joints[-1]-3.1415926])
      for i in tf_map_flange_list:
         for init_joint in init_jonints_list:
            check_joint = kinematic_solver.compute_best_ik(SE3(i), init_joint)
            if not check_joint:
               #self.logger.info("初始 init joint 求解失败") 
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
         self.logger.info("目标点ik仍无解")                  
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
class DFS:
   def __init__(self) -> None:

      # 初始化一个字典用于存储已访问的状态
      self.visited_states = {}
      # 日志
      self.logger = None
      # 超时
      self.time_out = None


   def find_all_paths(self, our_robot, check_robot_list, planning_env, init_joints, container_items, filtered_items, pick_items):
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


      stack = Stack()
      #初始化所有路径列表
      all_paths = []
      
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
               return all_paths
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
               new_state_key = state_key + [next_item.name]
               next_item_id = next_item.additional_info.values[-3]
               #判断箱子是否干涉
               if not check_collision(our_robot, check_robot_list,planning_env, init_joints, next_item):

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

class A_STAR():

   def __init__(self) -> None:

      #缓存来存储已计算的启发式代价
      self.heuristic_cache = {}
      # 初始化一个字典用于存储已访问的状态
      self.visited_states = {}
      self.logger = None
      # self.my_queue = queue.Queue()
      # # 创建事件对象
      # self.event = threading.Event()

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

      #对角线距离

      dx = abs(start_node.origin.x - target_node.origin.x)
      dy = abs(start_node.origin.y - target_node.origin.y)
      dz = abs(start_node.origin.z - target_node.origin.z)

      # 在水平、垂直和垂直方向上的移动成本
      diagonal_cost = min(dx, dy, dz)

      # 剩余移动成本
      remaining_cost = abs(dx - dy) + abs(dy - dz) + abs(dz - dx)

      # 计算总成本
      total_cost = int(diagonal_cost * math.sqrt(3) + remaining_cost)

         
      # 将计算结果放入缓存
      self.heuristic_cache[(start_node_id, target_node_id)] = total_cost

      return total_cost

   def check_all_collisions(self,our_robot, check_robot_list, planning_env, 
                            init_joints, next_item,open_set,state_key,pick_items,current_node,retrieved_item,g_cost,open_set_lock,stop_event):
      
      if stop_event.is_set():
         return False
      # 判断箱子是否干涉
      if not check_collision(our_robot, check_robot_list, planning_env, init_joints, next_item,):
         # 不干涉条件满足，节点ID到路径里
         new_state_key = state_key + [next_item.name]
         # 计算总代价
         for target_item in pick_items:

            #计算预估代价
            h_cost = self.heuristic_cost_estimate(next_item, target_item)
            #更新当前g(n)代价
            g_cost = self.g_current_cost(current_node,next_item)+g_cost
            #更新将f(n)代价
            f_cost = g_cost + h_cost

            # 将f(n)代价，当前代价g(n)，当前节点，路径ID,环境,箱子放入队列
            cost_item = CostItem(f_cost,g_cost,next_item,new_state_key,retrieved_item)  
            with open_set_lock:
               open_set.put(cost_item)   



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

      # 创建一个锁
      open_set_lock = threading.Lock()

      with concurrent.futures.ThreadPoolExecutor(max_workers=100) as executor:
         # 进入优先队列的循环,队列不为空就一直循环   
         while not found_shortest_path and not self.event.is_set():
            try:            
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
               # 过滤非顶层箱子
               remaining_items = filter_layer_items(remaining_items)


               # 搜索剩余可抓箱子是否与环境干涉
               for next_item in remaining_items:
                  executor.submit(self.check_all_collisions, our_robot, check_robot_list, planning_env, 
                  init_joints, next_item,open_set,state_key,pick_items,current_node,retrieved_item,g_cost,open_set_lock)
            except:
               queue.Empty
               break     
         self.my_queue.put(all_paths) 
         return all_paths  


   def path_check(self,path,container_items,our_robot,check_robot_list,planning_env,init_joints):
      #偏置
      slide_list = [[0.08, 0.08, 0.07, 0, 0, 0, 1], [0.08, -0.08, 0.07, 0, 0, 0, 1], [-0.08, 0.08, 0.07, 0, 0, 0, 1], [-0.08, -0.08, 0.07, 0, 0, 0, 1]]
      # 遍历路径
      self.logger(f"开始后验路径{path}")
      for node in path:
         self.logger(f"后验节点{node}")
         #获取当前节点箱子
         pick_item = list(filter(lambda x: x.additional_info.values[-3] == node, container_items))
         if len(pick_item)!=1:
            self.logger(f"后验检查,{node}不在容器中")
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
         
         #拆分xy偏移精度
         def split_slide(slide,precision):
            split_x_length = int(slide[0]/precision)
            split_y_length = int(slide[1]/precision)
            new_slide_x = []
            new_slide_y = []
            for i in range(1,abs(split_x_length)+1):
               if split_x_length>0:
                  if i*precision>=0.014:
                     new_slide_x.append(i*precision)
               else:
                  if -i*precision<=-0.014:
                     new_slide_x.append(-i*precision)        
            for i in range(1,abs(split_y_length)+1):
               if split_y_length>0:
                  if i*precision>=0.014:
                     new_slide_y.append(i*precision)
               else:
                  if -i*precision<=-0.014:
                     new_slide_y.append(-i*precision)   
            return_list = [[x,y]+slide[2:7] for x,y in zip(new_slide_x,new_slide_y)]                               
            return return_list  
         
         def divide_float_into_list(value, precision):
            # 计算需要多少步骤来从0达到目标值，加1是为了包含最终值本身
            steps = int(value / precision) + 1
            # 生成列表，使用round确保浮点数精度问题不会影响结果
            result = [round(i * precision, 2) for i in range(1, steps)]
            return result 
                                   
         #遍历偏置列表
         slide_flag = False
         for slide in slide_list:
            #xy偏移拆分精度
            precision = 0.001
            check_offset_xy_list = split_slide(slide,precision)
            self.logger(f"check_offset_xy_list is {check_offset_xy_list}")
            xy_check_collision = False
            for check_slide_xy in check_offset_xy_list:
               #更新偏移
               check_item = copy.deepcopy(pick_item)
               check_item.origin = Pose(*((SE3(check_slide_xy)*SE3(pose_to_list(check_item.origin))).xyz_quat))
               if check_collision(our_robot,check_robot_list,planning_env,init_joints,check_item):
                  self.logger(f"后验xy拆分{check_slide_xy}检测到碰撞")
                  xy_check_collision = True
                  break
               else:
                  continue
            if xy_check_collision:
               self.logger(f"偏移点{slide}在XY方向上检测到碰撞")   
               continue
            else:                                  
               #如果xy偏置无干涉,开始后验z方向的偏置   
               #z方向偏移拆分精度
               offset_z = slide[2]+0.23
               z_precision = 0.03
               offset_list = divide_float_into_list(offset_z,z_precision)
               self.logger(f"offset_list为{offset_list}")
               offset_check_slide_list = []
               for offset in offset_list:
                  append_check_slide = copy.deepcopy(slide)
                  append_check_slide[2] = offset 
                  offset_check_slide_list.append(append_check_slide)    
               self.logger(f"检查偏移点向上偏移,偏移列表为{offset_check_slide_list}")      
               z_check_collision = False    
               for check_slide_z in offset_check_slide_list:  
                  #更新偏移
                  check_item = copy.deepcopy(pick_item)
                  check_item.origin = Pose(*((SE3(check_slide_z)*SE3(pose_to_list(check_item.origin))).xyz_quat))    
                  check_item.origin.z+=0.035                 
                  if check_collision(our_robot,check_robot_list,planning_env,init_joints,check_item):
                     self.logger(f"后验z拆分{check_slide_z}检测到碰撞")
                     z_check_collision = True
                     break     
                  else:
                     continue     
               if z_check_collision:
                  self.logger(f"偏移点{slide}在Z方向上检测到碰撞")   
                  continue     
               else:   
                  self.logger(f"偏移点{slide}XYZ后验无碰撞")
                  slide_flag = True
                  break         
               
         if slide_flag:
            self.logger(f"{node}验证ok,偏置{slide}无碰撞,继续搜索")
            continue
         else:
            self.logger(f"{node}验证失败")
            our_robot.detach_object()   
            for check_robot in check_robot_list:
               check_robot.detach_object()              
            return False 
          
      our_robot.detach_object()   
      for check_robot in check_robot_list:
         check_robot.detach_object()                 
      return True       


   def a_star_search(self,our_robot,check_robot_list,planning_env,init_joints,container_items,filtered_items,pick_items):
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

      Returns:
      返回一个列表，列表按抓取顺序排列
      """
      
      # 初始化优先队列
      open_set = PriorityQueue() 

      # 初始化所有路径列表
      all_paths = [] 

      # 初始化标志变量
      found_shortest_path = False

      # copy了一份初始解的环境，便于后续环境里箱子的操作
      init_planning_env = copy.copy(planning_env)

      self.logger(f"计算初始可抓取箱子为{[i.additional_info.values[-3] for i in filtered_items]}")
      
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

      with concurrent.futures.ThreadPoolExecutor(max_workers=100) as executor:
         # 进入优先队列的循环,队列不为空就一直循环   
         while not found_shortest_path:
            try:            
               # 取出所有的栈数据
               retrieved_item = open_set.get()
               f_cost = retrieved_item.f_cost
               g_cost = retrieved_item.g_cost
               current_node = retrieved_item.current_node
               state_key = retrieved_item.state_key
               
               # 如果箱子在目标箱子里，则添加到所有路径里
               if current_node in pick_items:
                  self.logger(f"优先队列剩余数量{open_set.qsize()}")
                  # 搜索结束，回溯路径
                  all_paths.append(retrieved_item.reconstruct_path())
                  # 设置标志变量，告知找到最短路径
                  # found_shortest_path = True
                  # stop_event.set()
                  # break
                  # 初始化环境
                  planning_env = copy.copy(init_planning_env)
                  if self.path_check(all_paths[0],container_items,our_robot,check_robot_list,planning_env,init_joints):
                     # 设置标志变量，告知找到最短路径
                     found_shortest_path = True
                     stop_event.set()
                     break
                  else:
                     self.logger(f"已搜索到路径,但是路径不可行,重新搜索")
                     all_paths = []
                     continue

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
               # 过滤非顶层箱子
               remaining_items = filter_layer_items(remaining_items)


               # 搜索剩余可抓箱子是否与环境干涉
               for next_item in remaining_items:
                  executor.submit(self.check_all_collisions, our_robot, check_robot_list, planning_env, 
                  init_joints, next_item,open_set,state_key,pick_items,current_node,retrieved_item,g_cost,open_set_lock,stop_event)
            except:
               queue.Empty
               break   
         # 关闭线程池，不再接受新任务
         executor.shutdown(wait=False)     
         return all_paths  




def all_direction_items(planning_env,container_items,current_direction):
   """
   获取所有转向的信息

   Parameters:
   - planning_env: 当前转向的环境
   - container_items: 当前转向的箱子信息
   - current_direction: 当前转向

   Returns:
   返回一个字典,字典key为转向,value为转向的环境,箱子
   """

   #agv托盘面对应的角度值，通过这个度数可以知道转到那个面需要转多少度
   if current_direction == "A":
      all_direction_angle = {
            "A":0,
            "B":-90,
            "C":180,
            "D":90
      }
   elif current_direction == "B":
      all_direction_angle = {
            "B":0,
            "A":90,
            "C":-90,
            "D":180
      }           
   elif current_direction == "C":
      all_direction_angle = {
            "C":0,
            "A":180,
            "B":90,
            "D":-90
      }    
   elif current_direction == "D":
      all_direction_angle = {
            "D":0,
            "A":-90,
            "B":180,
            "C":90,
      } 

   all_direction_items_dict = {}

   #遍历所有面
   for direction,angle in all_direction_angle.items():
      #计算需要旋转的度数
      rotation_angle = angle

      #获取托盘旋转rotation 托盘坐标
      tf_space_rotation = SE3(tfm.quaternion_from_euler(0,0,np.deg2rad(rotation_angle)))
      tf_base_space = planning_env.get_workspace_ros("0").get_bottom_pose()    

      #初始箱子和环境
      new_container_items = copy.deepcopy(container_items)
      new_planning_env = copy.deepcopy(planning_env)

      #旋转箱子，是根据托盘坐标系左乘
      for container_item in new_container_items:
         tf_base_obj = SE3(pose_to_list(container_item.origin))
         tf_space_obj = (tf_base_space.inv())*tf_base_obj
         tf_new_space_obj = tf_space_rotation*tf_space_obj
         tf_new_base_obj = tf_base_space*tf_new_space_obj
         container_item.origin = Pose(*tf_new_base_obj.xyz_quat)

      #删除环境箱子，并添加新的箱子   
      new_planning_env.clear_container_all_items("0")
      new_planning_env.add_container_items("0",new_container_items)  
      
      all_direction_items_dict[direction] = {}
      all_direction_items_dict[direction]["container_items"] = new_container_items
      all_direction_items_dict[direction]["planning_env"] = new_planning_env

   return all_direction_items_dict        
            




def execute(self, inputs, outputs, gvm):
   self.logger.info("Hello {}".format(self.name))
   seach_current_time = time.time()
   # if not gvm.get_variable("motion_payload", per_reference=True, default=None):
   #    planning_env_msg = get_planning_environment()
   #    planning_env = PlanningEnvironmentRos.from_ros_msg(planning_env_msg)
   # else:
   #    last_payload = gvm.get_variable("motion_payload", per_reference=True, default=None)
   #    planning_env = PlanningEnvironmentRos.from_ros_msg(last_payload["planning_environment"])
   planning_env_msg = get_planning_environment()
   planning_env = PlanningEnvironmentRos.from_ros_msg(planning_env_msg)
   planning_env.remove_all_full_padding()
   planning_env.add_full_padding_exclude(["0"])      
   
   pick_ws_id = self.smart_data["pick_id"]

            
   #记录计算的次数
   global check_collision_num 
   check_collision_num = 0
   

   #获取环境箱子   
   container_items = planning_env.get_container_items(pick_ws_id) 
   if not container_items:
      raise Exception("理论上不该为空")
   #计算所有抓取姿态法兰joint
   q0 = inputs["q0"]
   if q0:
      init_joints = q0
   else:
      self.logger.info("未收到input q0 通过机器人获取实时init joint")
      r = RobotDriver(0)
      init_joints = list(r.get_joints()) 

   all_robot_states = AllRobotToolStatesRos.from_ros_msg(get_all_robot_tool_states()) 
   our_robot = all_robot_states.get_robot_ros("0")
   our_robot.detach_object()      
   #得到初始的夹具 
   robot_tool_state = all_robot_states.get_robot_tool_state("0")
   tool_name = robot_tool_state.tool_name
   init_cur_tool = all_robot_states.get_tool_ros(tool_name).to_ros_msg()   

   #通过碰撞体名字找到对应碰撞体
   init_tool_collisions = []
   init_clamp_collision = []
   tool_check_collision_list = ["+y","-y","x"]
   for i in init_cur_tool.tool_collisions_list:
      if i.name in tool_check_collision_list:
         init_tool_collisions.append(i)
      else:
         init_clamp_collision.append(i)  

   #相机扩大一点
   for i in init_clamp_collision:
      if i.name == "camera":
         i.primitives[0].dimensions = [0.6, 0.33+0.05, 0.25+0.05] 
      if i.name == "body":
         i.primitives[0].dimensions = [0.388, 0.588, 0.65]   
         #i.primitives[0].relative_pose.x-= 0
   #手动设置夹具两种姿态，这个是为了算碰撞避免每次set tool
   #共有 x+y和x-y的两种our robot    
   check_robot_list = []   
   
   #x,+y our robot
   check_our_robot = all_robot_states.get_robot_ros("0") 
   check_our_robot.detach_object()   
   check_our_robot_msg = check_our_robot.to_ros_msg()

   #添加默认就有的干涉物，像相机，夹具本体
   check_our_robot_msg.tool.tool_collisions.primitives=[i.primitives[0] for i in init_clamp_collision] 

   #添加可活动气缸
   for index,clamp_collision in enumerate(init_tool_collisions):
      if clamp_collision.name =="x":
         check_our_robot_msg.tool.tool_collisions.primitives.append(clamp_collision.primitives[0])
      if clamp_collision.name == "+y":
         check_our_robot_msg.tool.tool_collisions.primitives.append(clamp_collision.primitives[0])

   #因为our robot 不能用copy模块序列化，所以需要从新序列化        
   check_robot_list.append(RobotRos.from_ros_msg(check_our_robot_msg))

   #x,-y our robot
   check_our_robot = all_robot_states.get_robot_ros("0") 
   check_our_robot.detach_object()   
   check_our_robot_msg = check_our_robot.to_ros_msg()
   #添加默认就有的干涉物，像相机，夹具本体
         
   check_our_robot_msg.tool.tool_collisions.primitives=[i.primitives[0] for i in init_clamp_collision] 
   #添加可活动气缸
   for index,clamp_collision in enumerate(init_tool_collisions):
      if clamp_collision.name =="x":
         check_our_robot_msg.tool.tool_collisions.primitives.append(clamp_collision.primitives[0])
      if clamp_collision.name == "-y":
         check_our_robot_msg.tool.tool_collisions.primitives.append(clamp_collision.primitives[0])

   #因为our robot 不能用copy模块序列化，所以需要从新序列化        
   check_robot_list.append(RobotRos.from_ros_msg(check_our_robot_msg))


   #获取每个面的环境信息
   current_direction = inputs["direction"]
   all_direction_items_dict = all_direction_items(planning_env,container_items,current_direction)  
   all_direction_items_dict = {current_direction:all_direction_items_dict[current_direction]}
   all_paths = []  

   # 所有路径
   if self.smart_data["Enable_A*"]: 
      Config_A = self.smart_data["Config_A*"]  
      if Config_A["multiprocess"]:

         #计算结果进程队列   
         calculation_queue = multiprocessing.Queue()
         #终止事件
         termination_event = multiprocessing.Event()    
         calculation_process_list = []
         calculation_flag = False
         first_flag = True
         #所有转向所有夹具姿态，添加到计算进程
         for direction,direction_items in all_direction_items_dict.items():

            container_items = direction_items["container_items"]
            planning_env = direction_items["planning_env"]

            #过滤当前箱子抓取碰撞的箱子
            filter_check_collision = lambda container_item: not check_collision(our_robot, check_robot_list,planning_env, init_joints, container_item)
            filtered_items = list(filter(filter_check_collision, container_items))  
            filtered_items = sorted(filtered_items,key=lambda x:int(x.additional_info.values[-3]),reverse=True)


            #wcs订单里有目标点的箱子,通过非get vision results对箱子赋值，除了缓存区的箱子都是目标箱子
            pick_items = list(filter(lambda x:x.additional_info.values[-1]!="1",container_items))
            pick_items = sorted(pick_items,key=lambda x:int(x.additional_info.values[-3]),reverse=True)
                      
            #添加进程函数参数   
            out_robot_msg = dill.dumps(our_robot.to_ros_msg())
            planning_env_msg = dill.dumps(planning_env.to_ros_msg())
            container_items_msg = dill.dumps(container_items)
            pick_items_msg = dill.dumps(pick_items)
            filtered_items_msg = dill.dumps(filtered_items)
            

            #设置三种不同夹具去计算
            async_check_robot_list = []
            async_check_robot_list.append(dill.dumps([x.to_ros_msg() for x in check_robot_list]))
            async_check_robot_list.append(dill.dumps([check_robot_list[0].to_ros_msg()]))
            async_check_robot_list.append(dill.dumps([check_robot_list[1].to_ros_msg()]))
            #async_check_robot = dill.dumps([x.to_ros_msg() for x in check_robot_list])

            for async_check_robot in async_check_robot_list:
               async_input = (out_robot_msg, async_check_robot, planning_env_msg, init_joints, \
                              container_items_msg, filtered_items_msg, pick_items_msg,direction,current_direction)

               #启动计算进程
               a_star_async = A_STAR_ASYNC()
               a_star_async.calculation_queue = calculation_queue
               a_star_async.termination_event = termination_event
               a_star_async.logger = self.logger.info
               calculation_process = multiprocessing.Process(target=a_star_async.async_a_star_search, args=async_input)
               calculation_process.start()
               calculation_process_list.append(calculation_process)    
               if first_flag:  
                  try:       
                     #获取进程队列结果   
                     calculation_dict = calculation_queue.get(timeout=0.5)  
                     self.logger.info("calculation get")
                     calculation_flag = True
                     break
                  except:
                     first_flag = False
                     continue   
            if calculation_flag:
               break
            
         if not calculation_flag:      
            try:       
               #获取进程队列结果   
               calculation_dict = calculation_queue.get(timeout=200)
            except:
               self.logger.info("计算超时")
               #设置终止事件，让其他进程计算结束
               termination_event.set() 
               self.logger.info("calculation time out")
               for process in calculation_process_list:
                  try:
                     process.join(timeout=6) 
                     if process.is_alive():
                        self.logger.info(f"{process.pid}进程卡死") 
                        process.kill() 
                  except Exception as e:
                     self.logger.info(f"{process.pid}进程异常") 
                     process.kill()
               time.sleep(2)              
               calculation_queue.close()
               termination_event.clear()
               raise XYZExceptionBase("10021", "计算超时")   
         #设置终止事件，让其他进程计算结束
         termination_event.set() 
         #等待其他进程计算结束
         self.logger.info("calculation_complete")
         termination_event.wait() 
         
         for process in calculation_process_list:
            try:
               process.join(timeout=1) 
               if process.is_alive():
                  self.logger.info(f"{process.pid}进程卡死") 
                  process.kill() 
            except Exception as e:
               self.logger.info(f"{process.pid}进程异常")  
               process.kill()  
             
           
         calculation_queue.close()
         termination_event.clear()
         #添加路径和托盘转向    
         all_paths = calculation_dict["all_paths"] 
         direction = calculation_dict["direction"]   
         
      else:
         search_direction = self.smart_data["Config_A*"]["search_direction"]
         direction_items = all_direction_items_dict[search_direction]     
         container_items = direction_items["container_items"]
         planning_env = direction_items["planning_env"] 

         #过滤当前箱子抓取碰撞的箱子
         filter_check_collision = lambda container_item: not check_collision(our_robot, check_robot_list,planning_env, init_joints, container_item)
         filtered_items = list(filter(filter_check_collision, container_items))  

         #wcs订单里有目标点的箱子,通过非get vision results对箱子赋值，除了缓存区的箱子都是目标箱子
         pick_items = list(filter(lambda x:x.additional_info.values[-1]!="1",container_items))     
         a_start = A_STAR()
         a_start.logger = self.logger.info
         all_paths = a_start.a_star_search(our_robot,check_robot_list,planning_env,init_joints,container_items,filtered_items,pick_items)       
         direction = search_direction
         self.logger.info(check_collision_num)
  
   else:
      DFS_config = self.smart_data["DFS_config"]
      if DFS_config["multiprocess"]:
         
         #计算结果进程队列   
         calculation_queue = multiprocessing.Queue()
         #终止事件
         termination_event = multiprocessing.Event()    

         #所有转向所有夹具姿态，添加到计算进程
         for direction,direction_items in all_direction_items_dict.items():

            container_items = direction_items["container_items"]
            planning_env = direction_items["planning_env"]

            #过滤当前箱子抓取碰撞的箱子
            filter_check_collision = lambda container_item: not check_collision(our_robot, check_robot_list,planning_env, init_joints, container_item)
            filtered_items = list(filter(filter_check_collision, container_items))  


            #wcs订单里有目标点的箱子,通过非get vision results对箱子赋值，除了缓存区的箱子都是目标箱子
            pick_items = list(filter(lambda x:x.additional_info.values[-1]!="1",container_items))
            

            #添加进程函数参数   
            out_robot_msg = dill.dumps(our_robot.to_ros_msg())
            planning_env_msg = dill.dumps(planning_env.to_ros_msg())
            container_items_msg = dill.dumps(container_items)
            pick_items_msg = dill.dumps(pick_items)
            filtered_items_msg = dill.dumps(filtered_items)
            

            #设置三种不同夹具去计算
            async_check_robot_list = []
            async_check_robot_list.append(dill.dumps([x.to_ros_msg() for x in check_robot_list]))
            async_check_robot_list.append(dill.dumps([check_robot_list[0].to_ros_msg()]))
            async_check_robot_list.append(dill.dumps([check_robot_list[1].to_ros_msg()]))

            calculation_process_list = []
            for async_check_robot in async_check_robot_list:
               
               async_input = (out_robot_msg, async_check_robot, planning_env_msg, init_joints, \
                              container_items_msg, filtered_items_msg, pick_items_msg,direction)
  
               #启动计算进程
               dfs_search =DFS_ASYNC()
               dfs_search.calculation_queue = calculation_queue
               dfs_search.termination_event = termination_event
               dfs_search.logger = self.logger.info
               dfs_search.time_out = DFS_config["serch_time_out"]

               calculation_process = multiprocessing.Process(target=dfs_search.find_all_paths, args=async_input)
               calculation_process.start()
               calculation_process_list.append(calculation_process)

         calculation_dict_list = []
         #获取进程队列结果      
         calculation_dict_list.append(calculation_queue.get())   
  
         #设置终止事件，让其他进程计算结束
         termination_event.set() 
         for process in calculation_process_list:
            process.join()  

         while not calculation_queue.empty():
            calculation_dict_list.append(calculation_queue.get())    
         #等待其他进程计算结束
         self.logger.info("calculation_complete")
         calculation_queue.close()
         termination_event.clear()
         sorted(calculation_dict_list, key=lambda x:x["all_paths"])
         calculation_dict = calculation_dict_list[0]
         #添加路径和托盘转向    
         all_paths = calculation_dict["all_paths"] 
         direction = calculation_dict["direction"]   
      else:
         search_direction = DFS_config["search_direction"]
         direction_items = all_direction_items_dict[search_direction]  
         container_items = direction_items["container_items"]
         planning_env = direction_items["planning_env"] 

         #过滤当前箱子抓取碰撞的箱子
         filter_check_collision = lambda container_item: not check_collision(our_robot, check_robot_list,planning_env, init_joints, container_item)
         filtered_items = list(filter(filter_check_collision, container_items))  

         #wcs订单里有目标点的箱子,通过非get vision results对箱子赋值，除了缓存区的箱子都是目标箱子
         pick_items = list(filter(lambda x:x.additional_info.values[-1]!="1",container_items))    
         
         dfs_search =DFS()
         dfs_search.logger = self.logger.info
         dfs_search.time_out = DFS_config["serch_time_out"]
         all_paths = dfs_search.find_all_paths(our_robot, check_robot_list, planning_env, init_joints, container_items, filtered_items, pick_items)   
         direction = search_direction
         self.logger.info(check_collision_num)

   #最短路径
   if not all_paths:
      self.logger.info(f"没有找到可以抓取到目标物无碰撞的路径解")
      raise Exception("当前路径所有箱子已遍历,但仍未找到目标点,是个bug")
      
   shortest_path_list = list(filter(lambda x:len(x)==len(min(all_paths,key=lambda x: len(x))),all_paths))
   self.logger.info(shortest_path_list)

   outputs["shortest_path"] = shortest_path_list
   gvm.set_variable("plan_path", shortest_path_list[0], per_reference=False)
   gvm.set_variable("direction", direction, per_reference=False)
   self.logger.info(f"search time is {time.time()-seach_current_time}")


   if direction==current_direction:
      self.logger.info(f"current_direction do not change")
      return "success"
   else:
      raise Exception("理论上这里不该有转向的解,是个bug")

