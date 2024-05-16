
# import sys
# path_list = ['', '/opt/ros/noetic/lib/python3/dist-packages', '/usr/lib/python38.zip', '/usr/lib/python3.8', '/usr/lib/python3.8/lib-dynload', '/home/xyz/.local/lib/python3.8/site-packages', '/usr/local/lib/python3.8/dist-packages', '/usr/lib/python3/dist-packages', '/usr/lib/python3.8/dist-packages']
# for path in path_list:
#     sys.path.append(path)
# from xyz_motion import PlanningEnvironmentRos
# import os


# os.environ["DISPLAY"] = ":1"
 
# # 要添加的路径
# additional_path = "/usr/local/bin:/opt/ros/noetic/bin:/opt/ros/noetic/lib/python3/dist-packages:/home/xyz/.local/bin:/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin:/usr/games:/usr/local/games:/snap/bin:/home/xyz/xyz_app/software/build/bin"
 
# # 获取当前的 PATH 环境变量值
# current_path = os.environ.get("PATH", "")
 
# # 添加要设置的路径到当前的 PATH 环境变量值中，使用 ":" 分隔
# new_path = f"{additional_path}:{current_path}"
 
# # 设置新的 PATH 环境变量值
# os.environ["PATH"] = new_path

# os.environ["LD_LIBRARY_PATH"] = "/opt/ros/noetic/lib:" + os.environ.get("LD_LIBRARY_PATH", "")


# planning_env = PlanningEnvironmentRos.from_json_file("/home/xyz/xyz_app/projects/dapeng_station_0/wcs_pallet_json/planning_env.json")
# for i in range(1,7):
#     planning_env.remove_workspace(str(i))
# primitive_map = planning_env.get_primitive_group_map()   
# col_list = []
# for i in primitive_map.keys():
#     col_list.append(i)
# for i in col_list:    
#     planning_env.remove_primitive_group(i)   
# # planning_env.remove_primitive_group("col_1")
# # planning_env.remove_primitive_group("col_2")
# # planning_env.remove_primitive_group("col_3")
# # planning_env.remove_primitive_group("col_4")
# # planning_env.remove_primitive_group("col_13")
# planning_env.show()   
