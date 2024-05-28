from ast import arg
from cgi import test
from distutils.log import debug
from xyz_env_manager.client import get_planning_environment
from xyz_motion import PlanningEnvironmentRos,pose_to_list,SE3
import copy,random
import numpy as np
import tf.transformations as tfm
from matplotlib import transforms
import matplotlib as mpl
import matplotlib.pyplot as plt
import matplotlib.patches as mpathes
from rafcon.xyz_exception_base import XYZLayoutException, XYZExceptionBase

def check_pose(pose):
        if (pose[3]=='W'and pose[2]=="l") or (pose[3]=='L'and pose[2]=="w"):
            point_0 = (SE3(tfm.euler_matrix(0,0,z_angles))*SE3(pose[0:2]+[0,0,0,0,1])).xyz_quat 
            point_1 = (SE3(point_0)*SE3([sku_info["width"],0,0,0,0,0,1])).xyz_quat  
            point_2 = (SE3(point_0)*SE3([0,sku_info["length"],0,0,0,0,1])).xyz_quat 
            point_3 = (SE3(point_0)*SE3([sku_info["width"],sku_info["length"],0,0,0,0,1])).xyz_quat 
        elif(pose[3]=='W'and pose[2]=="w") or (pose[3]=='L'and pose[2]=="l"):
            point_0 = (SE3(tfm.euler_matrix(0,0,z_angles))*SE3(pose[0:2]+[0,0,0,0,1])).xyz_quat  
            point_1 = (SE3(point_0)*SE3([sku_info["length"],0,0,0,0,0,1])).xyz_quat  
            point_2 = (SE3(point_0)*SE3([0,sku_info["width"],0,0,0,0,1])).xyz_quat 
            point_3 = (SE3(point_0)*SE3([sku_info["length"],sku_info["width"],0,0,0,0,1])).xyz_quat      
        check_list = list(filter(lambda x:abs(x[0])>(space_dimensions[0]/2) or abs(x[1])>(space_dimensions[1]/2) ,[point_0,point_1,point_2,point_3]))
        if not len(check_list)==0:
            return True
        else:
            return False   

def show_bug(debug_pose,text_pose,sku_info,space_dimensions,title_name,z_angles=None,x_angles_180=None,y_angles_180=None):
    fig,ax = plt.subplots()
    for key,item in debug_pose.items():
        pose = np.array([item[0],item[1]]) 
        if (item[3]=='W'and item[2]=="l") or (item[3]=='L'and item[2]=="w"):     
            if x_angles_180:
                pose[1] = -pose[1]-sku_info["length"]
            elif y_angles_180:
                pose[0] = -pose[0]-sku_info["width"]  
            box = mpathes.Rectangle(pose,sku_info["width"],sku_info["length"],color=random.choice(["r","g","b"]),alpha=random.random(),lw=3) 
        elif (item[3]=='W'and item[2]=="w") or (item[3]=='L'and item[2]=="l"):
            if x_angles_180:
                pose[1] = -pose[1]-sku_info["width"]
            if y_angles_180:
                pose[0] = -pose[0]-sku_info["length"]          
            box = mpathes.Rectangle(pose,sku_info["length"],sku_info["width"],color=random.choice(["r","g","b"]),alpha=random.random(),lw=3)   
        if z_angles or y_angles_180 or x_angles_180:
            if z_angles:
                base = plt.gca().transData
                rot = transforms.Affine2D().rotate_deg(np.rad2deg(z_angles))
                box.set_transform(rot + base) 
            text = text_pose[key+len(debug_pose)] 
            text_key = key+len(debug_pose)
        else:    
            text = text_pose[key]  
            text_key = key   
        ax.add_patch(box)   
        ax.text(text[0],text[1],str(text_key))                  
    ax.set_ylabel("Y")
    ax.set_xlabel("X")  
    ax.axis([-space_dimensions[0]/2,space_dimensions[0]/2,-space_dimensions[1]/2,space_dimensions[1]/2])    
    ax.set_title(title_name)      
    plt.show()
    
    
def dot_product_angle(v1, v2):
    if np.linalg.norm(v1) == 0 or np.linalg.norm(v2) == 0:
        pass
    else:
        vector_dot_product = np.dot(v1, v2)
        arccos = np.arccos(vector_dot_product / (np.linalg.norm(v1) * np.linalg.norm(v2)))
        angle = np.degrees(arccos)
        return angle
    return 0
    
    
def execute(self, inputs, outputs, gvm):
    self.logger.info("Hello {}".format(self.name))
    
    planning_env_msg = get_planning_environment()
    planning_env = PlanningEnvironmentRos.from_ros_msg(planning_env_msg)    
        
    space_env = planning_env.get_workspace_ros(self.smart_data["space_id"])

    tf_base_space = space_env.get_bottom_pose()
    global space_dimensions
    space_dimensions = space_env.get_dimensions()
    
    input_sku_info = inputs["sku_info"]
    global sku_info
    sku_info = self.smart_data["sku_info"]
    
    if input_sku_info:
        sku_info["length"] = input_sku_info["length"]
        sku_info["width"] = input_sku_info["width"]
        sku_info["height"] = input_sku_info["height"]
        sku_info["weight"] = input_sku_info["weight"]
    
    if self.smart_data["type"]['place_pose_by_size']:
        from xyz_logistics_hmi_back.utils.design import query_pallet_design_by_size
        pallet_design_data = query_pallet_design_by_size(1000*sku_info["length"], 1000*sku_info["width"], 1000*sku_info["height"],"1")
        if not pallet_design_data:
            raise XYZExceptionBase("60007", "Pallet Design from HMI is empty.")
        pallet_design = pallet_design_data["layout"]
        odd_poses = np.array(pallet_design[0])
        odd_poses[:, :3] /= 1000.0
        even_poses = np.array(pallet_design[1])
        even_poses[:, :3] /= 1000.0
        
        pose_dict = {}
        debug_pose = {}
        for index,item in enumerate(odd_poses):
            append_item = item[0:3].tolist()
            append_item[2] = sku_info["height"]
            pose_dict[index+1] = append_item
            pose_x_angle = dot_product_angle(SE3(item).rotation[0],SE3([0,0,0,0,0,0,1]).rotation[0])
            if 20>pose_x_angle or pose_x_angle>160:
                debug_pose[index+1] = [append_item[0]-sku_info["length"]/2,append_item[1]-sku_info["width"]/2,"l","L"]
            else:
                debug_pose[index+1] = [append_item[0]-sku_info["width"]/2,append_item[1]-sku_info["length"]/2,"w","L"]
    elif self.smart_data["type"]['place_pose_by_scan_code']:    
        scan_code = inputs["scan_code"]
        scan_code = gvm.get_variable("depal_scan_code", per_reference=False, default=None)
        from xyz_logistics_hmi_back.utils.design import query_pallet_design_by_scan_code
        pallet_design_data = query_pallet_design_by_scan_code(scan_code, "1")
        if not pallet_design_data:
            raise XYZExceptionBase("60007", "Pallet Design from HMI is empty.")        
        pallet_design = pallet_design_data["layout"]
        odd_poses = np.array(pallet_design[0])
        odd_poses[:, :3] /= 1000.0
        even_poses = np.array(pallet_design[1])
        even_poses[:, :3] /= 1000.0
        
        pose_dict = {}
        debug_pose = {}
        for index,item in enumerate(odd_poses):
            append_item = item[0:3].tolist()
            append_item[2] = sku_info["height"]
            pose_dict[index+1] = append_item
            pose_x_angle = dot_product_angle(SE3(item).rotation[0],SE3([0,0,0,0,0,0,1]).rotation[0])
            if 20>pose_x_angle or pose_x_angle>160:
                debug_pose[index+1] = [append_item[0]-sku_info["length"]/2,append_item[1]-sku_info["width"]/2,"l","L"]
            else:
                debug_pose[index+1] = [append_item[0]-sku_info["width"]/2,append_item[1]-sku_info["length"]/2,"w","L"]
    
    else:    
        guillotine_packing_cfg = self.smart_data["type"]["guillotine_packing_cfg"]
        no_guillotine_packing_cfg = self.smart_data["type"]["no_guillotine_packing_cfg"]
        if (guillotine_packing_cfg["guillotine_packing"] and no_guillotine_packing_cfg["no_guillotine_packing"]) or\
            (not guillotine_packing_cfg["guillotine_packing"] and not no_guillotine_packing_cfg["no_guillotine_packing"]):
            raise Exception ("选择垛型模式上设置错误，必须有且只有一个开启")

        if guillotine_packing_cfg["guillotine_packing"]:
            pass

        if no_guillotine_packing_cfg["no_guillotine_packing"]:
            space_config = no_guillotine_packing_cfg["space_config"]
            side_config = space_config["side_config"]
            box_id = 0
            if space_config["side"]=="L":
                point_0 = [-space_dimensions[0]/2,-space_dimensions[1]/2,sku_info["height"]]
                point_1 = [space_dimensions[0]/2,-space_dimensions[1]/2,sku_info["height"]]
                pose_dict = {}
                debug_pose = {}
                for index,item in enumerate(side_config):
                    if item.count("l"):
                        if item.count("l")!=len(item):
                            raise "side config fault"
                        for i in item:
                            point_0[1] = point_1[1]
                            for width_index in range(int(space_dimensions[1]//sku_info["width"])):
                                box_id+=1
                                if no_guillotine_packing_cfg["center"]:
                                    center_offset_y = (space_dimensions[1]-space_dimensions[1]//sku_info["width"]*sku_info["width"])/2
                                    pose_dict[box_id] = [point_0[0]+sku_info["length"]/2,point_0[1]+sku_info["width"]/2+center_offset_y,sku_info["height"]]
                                    debug_pose[box_id] = [point_0[0],point_0[1]+center_offset_y,"l","L"]
                                else:    
                                    pose_dict[box_id] = [point_0[0]+sku_info["length"]/2,point_0[1]+sku_info["width"]/2,sku_info["height"]]
                                    debug_pose[box_id] = [point_0[0],point_0[1],"l","L"]
                                point_0[1]+=sku_info["width"]
                            point_0[0] +=sku_info["length"] 
                            if point_0[0]>point_1[0]:
                                raise "side_config 长边配置错误,已超出长边"                                
                    elif item.count("w"):
                        if item.count("w")!=len(item):
                            raise "side config fault"
                        for i in item:
                            point_0[1] = point_1[1]
                            for width_index in range(int(space_dimensions[1]//sku_info["length"])):
                                box_id+=1
                                if no_guillotine_packing_cfg["center"]:
                                    center_offset_y = (space_dimensions[1]-space_dimensions[1]//sku_info["length"]*sku_info["length"])/2       
                                    pose_dict[box_id] = [point_0[0]+sku_info["width"]/2,point_0[1]+sku_info["length"]/2+center_offset_y,sku_info["height"]]
                                    debug_pose[box_id] = [point_0[0],point_0[1]+center_offset_y,"w","L"]
                                else:
                                    pose_dict[box_id] = [point_0[0]+sku_info["width"]/2,point_0[1]+sku_info["length"]/2,sku_info["height"]]    
                                    debug_pose[box_id] = [point_0[0],point_0[1],"w","L"] 
                                point_0[1]+=sku_info["length"]
                            point_0[0] +=sku_info["width"] 
                            if point_0[0]>point_1[0]:
                                raise "side_config 长边配置错误，已超出长边"                                                                             
                    else:
                        raise "side config fault"  
                if no_guillotine_packing_cfg["center"]:
                    center_offset_x = (point_1[0]-point_0[0])/2
                    for key,item in pose_dict.items():
                        item[0] = item[0]+center_offset_x
                        pose_dict[key] = item  
                    for key,item in debug_pose.items():
                        item[0] = item[0]+center_offset_x
                        debug_pose[key] = item          
                                    
                    
            elif space_config["side"]=="W":
                point_0 = [-space_dimensions[0]/2,-space_dimensions[1]/2,sku_info["height"]]
                point_1 = [-space_dimensions[0]/2,space_dimensions[1]/2,sku_info["height"]]
                pose_dict = {}
                debug_pose = {}
                for index,item in enumerate(side_config):
                    if item.count("l"):
                        if item.count("l")!=len(item):
                            raise "side config fault"
                        for i in item:
                            point_0[0] = point_1[0]
                            for length_index in range(int(space_dimensions[0]//sku_info["width"])):
                                box_id+=1
                                if no_guillotine_packing_cfg["center"]:
                                    center_offset_x = (space_dimensions[0]-space_dimensions[0]//sku_info["width"]*sku_info["width"])/2   
                                    pose_dict[box_id] = [point_0[0]+sku_info["width"]/2+center_offset_x,point_0[1]+sku_info["length"]/2,sku_info["height"]]
                                    debug_pose[box_id] = [point_0[0]+center_offset_x,point_0[1],'l','W']
                                else:     
                                    pose_dict[box_id] = [point_0[0]+sku_info["width"]/2,point_0[1]+sku_info["length"]/2,sku_info["height"]]
                                    debug_pose[box_id] = [point_0[0],point_0[1],'l','W']
                                point_0[0]+=sku_info["width"]
                            point_0[1] +=sku_info["length"] 
                            if point_0[1]>point_1[1]:
                                raise "side_config 长边配置错误,已超出长边"                                
                    elif item.count("w"):
                        if item.count("w")!=len(item):
                            raise "side config fault"
                        for i in item:
                            point_0[0] = point_1[0]
                            for length_index in range(int(space_dimensions[0]//sku_info["length"])):
                                box_id+=1
                                if no_guillotine_packing_cfg["center"]:
                                    center_offset_x = (space_dimensions[0]-space_dimensions[0]//sku_info["length"]*sku_info["length"])/2   
                                    pose_dict[box_id] = [point_0[0]+sku_info["length"]/2+center_offset_x,point_0[1]+sku_info["width"]/2,sku_info["height"]]
                                    debug_pose[box_id] = [point_0[0]+center_offset_x,point_0[1],'w','W']
                                else:       
                                    pose_dict[box_id] = [point_0[0]+sku_info["length"]/2,point_0[1]+sku_info["width"]/2,sku_info["height"]]
                                    debug_pose[box_id] = [point_0[0],point_0[1],'w','W']
                                point_0[0]+=sku_info["length"]
                            point_0[1] +=sku_info["width"] 
                            if point_0[1]>point_1[1]:
                                raise "side_config 长边配置错误，已超出长边"                                                                             
                    else:
                        raise "side config fault"   
                if no_guillotine_packing_cfg["center"]:
                    center_offset_y = (point_1[1]-point_0[1])/2
                    for key,item in pose_dict.items():
                        item[1] = item[1]+center_offset_y
                        pose_dict[key] = item  
                        
                    for key,item in debug_pose.items():
                        item[1] = item[1]+center_offset_y
                        debug_pose[key] = item      
            else:
                raise "space_config 长短边选择配置错误"                                                

        
    if self.smart_data["show_debug"]:
        show_bug(debug_pose,pose_dict,sku_info,space_dimensions,"odd_pose_id")  
    global z_angles 
    global x_angles_180 
    global y_angles_180      
    z_angles = np.deg2rad(self.smart_data["mirror"]["z_angles"])
    x_angles_180 = self.smart_data["mirror"]["x_angles_180"]
    y_angles_180 = self.smart_data["mirror"]["y_angles_180"]
    text_pose_1 = {}
    if z_angles:
        if x_angles_180 or y_angles_180:
            raise "偶数层旋转配置错误"
        for key,item in pose_dict.items():
            text_pose_1[key+len(debug_pose)+1] = (SE3(tfm.euler_matrix(0,0,z_angles))*SE3(pose_dict[key][0:3]).xyz_quat[0:3]).tolist()        
    elif x_angles_180:
        if z_angles or y_angles_180:
            raise "偶数层旋转配置错误"        
        for key,item in pose_dict.items():
            text_pose_1[key+len(debug_pose)+1] = (SE3(tfm.euler_matrix(3.1415926,0,0))*SE3(pose_dict[key][0:3]).xyz_quat[0:3]).tolist()  
    elif y_angles_180:
        if z_angles or x_angles_180:
            raise "偶数层旋转配置错误"         
        for key,item in pose_dict.items():
            text_pose_1[key+len(debug_pose)+1] = (SE3(tfm.euler_matrix(0,3.1415926,0))*SE3(pose_dict[key][0:3]).xyz_quat[0:3]).tolist()  
    elif not x_angles_180 and not y_angles_180 and not z_angles:
        pass
    else:    
        raise "偶数层旋转配置错误"        
           
                        
    if z_angles or x_angles_180 or y_angles_180:
        if self.smart_data["show_debug"]:
            show_bug(debug_pose,text_pose_1,sku_info,space_dimensions,"even_pose_id",z_angles,x_angles_180,y_angles_180)  
        check_list = [i for i in debug_pose.values()]
        if list(filter(check_pose,check_list)):
            raise "偶数层旋转超出托盘范围"     
              
    overlapping_heihgt = self.smart_data["overlapping_heihgt"]    
    overlapping_heihgt = inputs["sku_info"]["overlapping_heihgt"] 
    
    layer_num = 1+int((space_dimensions[2]-sku_info["height"])//(sku_info["height"]-overlapping_heihgt))
    new_pose_dict = copy.copy(pose_dict)
    for layer in range(1,layer_num):    
        for pose_id,pose_item in pose_dict.items():
            append_pose = copy.copy(pose_item)
            append_pose[2]+=(sku_info["height"]-overlapping_heihgt)*layer
            
            if layer%2:
                if x_angles_180:
                    append_pose = SE3(tfm.euler_matrix(3.1415926,0,0))*SE3(append_pose).xyz_quat[0:3]  
                    append_pose[2] = -append_pose[2]
                elif y_angles_180:
                    append_pose = SE3(tfm.euler_matrix(0,3.1415926,0))*SE3(append_pose).xyz_quat[0:3]     
                    append_pose[2] = -append_pose[2]                  
                elif z_angles:
                    append_pose = SE3(tfm.euler_matrix(0,0,z_angles))*SE3(append_pose).xyz_quat[0:3]
            new_pose_dict[layer*len(pose_dict)+pose_id] = append_pose

    if self.smart_data["space_id"]=="0":
        all_pose_dict = {}
        all_pose_dict["C"] = new_pose_dict

        def transforms_pose_dict(pose_dict,angle,name):
            angle_trans = SE3(tfm.quaternion_from_euler(0,0,np.deg2rad(angle)))
            append_pose_dict = {}
            for idx,pose in pose_dict.items():
                pose = SE3(pose+[0,0,0,0])
                new_pose = (angle_trans*pose).xyz_quat[0:3]
                append_pose_dict[idx] = new_pose
            return append_pose_dict    
        all_pose_dict["A"] = transforms_pose_dict(new_pose_dict,180,"A")
        all_pose_dict["B"] = transforms_pose_dict(new_pose_dict,90,"B")
        all_pose_dict["D"] = transforms_pose_dict(new_pose_dict,-90,"D")
        outputs["all_pose_dict"] = all_pose_dict
        self.logger.info(f"拣配来料垛型ID为:{new_pose_dict}")
        gvm.set_variable("all_pose_dict", all_pose_dict, per_reference=False)     
        return "depal"
    elif self.smart_data["space_id"]=="1":
        outputs["pose_dict"] = new_pose_dict
        return "cache"  
    else:
        return "no_action"  
