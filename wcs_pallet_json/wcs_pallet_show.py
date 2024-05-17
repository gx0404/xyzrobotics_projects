from xyz_motion import PlanningEnvironmentRos
planning_env = PlanningEnvironmentRos.from_json_file("/home/xyz/xyz_app/projects/dapeng_station_0/wcs_pallet_json/planning_env.json")
for i in range(1,9):
    planning_env.remove_workspace(str(i))
    
primitive_name_list = []
primitive_map = planning_env.get_primitive_group_map()    
for i in primitive_map.keys():
    primitive_name_list.append(i)
for i in primitive_name_list:
    planning_env.remove_primitive_group(i)
planning_env.show()   
