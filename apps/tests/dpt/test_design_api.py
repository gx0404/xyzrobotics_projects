#  Copyright (c) XYZ Robotics Inc. - All Rights Reserved.
#  Unauthorized copying of this file, via any medium is strictly prohibited.
#  Proprietary and confidential.
#  Author: Shuanglong Wang <shuanglong.wang@xyzrobotics.ai>, 6. 2021.
#  Author: Xiaosheng Zhu <xiaosheng.zhu@xyzrobotics.ai>, 7. 2022.
# -*- coding: utf-8 -*-

import json
import unittest
import requests
import os 
import sys

FILE_PATH = os.path.abspath(__file__)
sys.path.append(os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(FILE_PATH)))))


from sqlalchemy import false
client = requests.session()


class TestDesignAPI(unittest.TestCase):
    """
    Test design api.
    """
    def test_design(self):
        # 新增托盘,为了垛型规划
        delete_pallet = True
        # 查询是否已经有符合条件的托盘
        #get
        #通过过滤条件查询托盘
        data =  {
                "length_min": "1200",
                "length_max": "1200",
                "width_min": "1100",
                "width_max": "1100"
                }			
        res = client.get('http://127.0.0.1:7002/api/dpt/pallets/', data=data)
        res_data = res.json()
        assert res_data["msg"] == "success"

        records_total = res_data["data"]["records_total"]
        if records_total == 0:
            # post
            # 新增托盘
            data = {
				"name": "xyz",
				"length": 1200.0,
				"width": 1100.0,
				"height": 150.0,
				"max_height": 1200.0,
				"max_weight": 500.0,
				"pallet_type": False
	            }			
            res = client.post('http://127.0.0.1:7002/api/dpt/pallets/', data=data)
            res_data = res.json()
            assert res_data["msg"] == "success"
            pallet_data = res_data["data"]
            pallet_id = pallet_data["id"]
            delete_pallet = True
        else:
            pallet_data = res_data["data"]
            pallet_id = pallet_data["list"][0]["id"]
            delete_pallet = False
        #get 
        # 根据id查询托盘信息
        res = client.get('http://127.0.0.1:7002/api/dpt/pallets/{}'.format(pallet_id))
        res_data = res.json()
        assert res_data["msg"] == "success"


        # 新增纸箱，为了码垛规划
        delete_box = True
        # 查询是否已经有符合条件的箱子
        #get
        #通过过滤条件查询纸箱
        data =  {
                "length_min": "300",
                "length_max": "300",
                "width_min": "200",
                "width_max": "200",
                "height_min": "100",
                "height_max": "100"
                }			
        res = client.get('http://127.0.0.1:7002/api/dpt/boxes/', data=data)
        res_data = res.json()
        assert res_data["msg"] == "success"

        records_total = res_data["data"]["records_total"]
        if records_total == 0:
            # post
            # 新增纸箱
            data = {
                    "name": "box123",
                    "length": 300.0,
                    "width": 200.0,
                    "height": 100.0,
                    "weight": 12.0,
                    "scan_code": "xyz123"
                    }			
            res = client.post('http://127.0.0.1:7002/api/dpt/boxes/', data=data)
            res_data = res.json()
            assert res_data["msg"] == "success"
            box_data = res_data["data"]
            box_id = box_data["id"]
            delete_box = True
        else:
            box_data = res_data["data"]
            box_id = box_data["list"][0]["id"]
            delete_box = False


        # get
        # 查询纸箱
        res = client.get('http://127.0.0.1:7002/api/dpt/boxes/{}'.format(box_id))
        res_data = res.json()
        assert res_data["msg"] == "success"

        # post
        # 生成参考垛型
        data =  {
                "pallet_id": pallet_id,
                "box_id": box_id,
                "guillotine_packing": 1,
                "barcode_direction": 1,
                "mirror": 0,
                "flip": "",
                "layers": 5
                }			
        res = client.post('http://127.0.0.1:7002/api/dpt/design/', data=data)
        res_data = res.json()
        assert res_data["msg"] == "success"

        layout_data = res_data["data"]
        # post
        # 新增垛型规划
        data = {
            "pallet_id": pallet_id,
            "box_id": box_id,
            "guillotine_packing": 1,
            "barcode_direction": 1,
            "mirror": 0,
            "flip": "",
            "layers": 5,
            "layout": layout_data
            }
        res = client.post('http://127.0.0.1:7002/api/dpt/design/', data=data)
        res_data = res.json()
        assert res_data["msg"] == "success"

        # get
        # 通过过滤条件查询垛型规划
        data =  {
                "length_min": "300",
                "length_max": "300",
                "width_min": "200",
                "width_max": "200",
                "height_min": "100",
                "height_max": "100"
                }		
        res = client.get('http://127.0.0.1:7002/api/dpt/design/', data=data)
        res_data = res.json()
        assert res_data["msg"] == "success"

        design_id = res_data["data"]["list"][0]["id"]

        # put
        # 修改垛型规划保存在数据库中的数据
        # 但是这个功能在HMI上其实是没有实际使用的，也不需要使用
        modify_data = {
                    "mirror": 1
                    }
        res = client.put('http://127.0.0.1:7002/api/dpt/design/{}'.format(design_id), data=modify_data)
        res_data = res.json()
        assert res_data["msg"] == "success"
        modify_data = {
                    "mirror": 0
                    }
        res = client.put('http://127.0.0.1:7002/api/dpt/design/{}'.format(design_id), data=modify_data)
        res_data = res.json()
        assert res_data["msg"] == "success"

        # get all
        # 查询所有垛型规划信息
        res = client.get('http://127.0.0.1:7002/api/dpt/design/')
        res_data = res.json()
        assert res_data["msg"] == "success"


        # XTF查询垛型规划数据
        data =  {
                "length": 300.0,
				"width": 200.0,
				"height": 100.0,
                "pallet_id":pallet_id
                }		
        res = client.get('http://127.0.0.1:7002/api/dpt/query_pallet_design/', data=data)
        res_data = res.json()
        assert res_data["msg"] == "success", res_data

        # delete
        # 删除垛型规划信息
        res = client.delete('http://127.0.0.1:7002/api/dpt/design/{}'.format(design_id))
        res_data = res.json()
        assert res_data["msg"] == "success"

        #get 
        # 根据id查询垛型规划信息
        res = client.get('http://127.0.0.1:7002/api/dpt/design/{}'.format(design_id))
        res_data = res.json()
        assert res_data["code"] == -1

        # 处理为了垛型规划而新增的纸箱
        if delete_box:
            # delete
            # 删除纸箱
            res = client.delete('http://127.0.0.1:7002/api/dpt/boxes/{}'.format(box_id))
            res_data = res.json()
            assert res_data["msg"] == "success"

            #get 
            # 根据id查询纸箱信息
            res = client.get('http://127.0.0.1:7002/api/dpt/boxes/{}'.format(box_id))
            res_data = res.json()
            assert res_data["code"] == -1
        
        # 处理为了垛型规划而新增的托盘
        if delete_pallet:
            # delete
            # 删除托盘
            res = client.delete('http://127.0.0.1:7002/api/dpt/pallets/{}'.format(pallet_id))
            res_data = res.json()
            assert res_data["msg"] == "success"

            #get 
            # 根据id查询托盘信息
            res = client.get('http://127.0.0.1:7002/api/dpt/pallets/{}'.format(pallet_id))
            res_data = res.json()
            assert res_data["code"] == -1
    
    """
    Test pallet api.
    """
    def test_pallet(self):
        # post
        # 新增托盘
        data =  {
				"name": "xyz",
				"length": 1500.0,
				"width": 1100.0,
				"height": 150.0,
				"max_height": 1200.0,
				"max_weight": 500.0,
				"pallet_type": False
                }   			
        res = client.post('http://127.0.0.1:7002/api/dpt/pallets/', data=data)
        res_data = res.json()
        assert res_data["msg"] == "success"

        # get
        # 查询托盘
        pallet_data = res_data["data"]
        pallet_id = pallet_data["id"]

        res = client.get('http://127.0.0.1:7002/api/dpt/pallets/{}'.format(pallet_id))
        res_data = res.json()
        assert res_data["msg"] == "success"

        # put
        # 修改托盘尺寸
        modify_data = {
		"length": 1200.0,
		"width": 1000.0
        }
        res = client.put('http://127.0.0.1:7002/api/dpt/pallets/{}'.format(pallet_id), data=modify_data)
        res_data = res.json()
        assert res_data["msg"] == "success"
        pallet_data = res_data["data"]
        assert pallet_data["width"] == 1000.0

        # get all
        # 查询所有托盘信息
        res = client.get('http://127.0.0.1:7002/api/dpt/pallets/')
        res_data = res.json()
        assert res_data["msg"] == "success"

        #get
        #通过过滤条件查询托盘
        data =  {
                "length_min": "1100",
                "length_max": "1300"
                } 			
        res = client.get('http://127.0.0.1:7002/api/dpt/pallets/', data=data)
        res_data = res.json()
        assert res_data["msg"] == "success"

        # delete
        # 删除托盘
        res = client.delete('http://127.0.0.1:7002/api/dpt/pallets/{}'.format(pallet_id))
        res_data = res.json()
        assert res_data["msg"] == "success"

        #get 
        # 根据id查询托盘信息
        res = client.get('http://127.0.0.1:7002/api/dpt/pallets/{}'.format(pallet_id))
        res_data = res.json()
        assert res_data["code"] == -1

    """
    Test box api.
    """
    def test_box(self):
        # post
        # 新增纸箱
        data = {
				"name": "box123",
				"length": 500.0,
				"width": 400.0,
				"height": 300.0,
				"weight": 12.0,
				"scan_code": "xyz123"
                }			
        res = client.post('http://127.0.0.1:7002/api/dpt/boxes/', data=data)
        res_data = res.json()
        assert res_data["msg"] == "success"

        # get
        # 查询纸箱
        box_data = res_data["data"]
        box_id = box_data["id"]
        res = client.get('http://127.0.0.1:7002/api/dpt/boxes/{}'.format(box_id))
        res_data = res.json()
        assert res_data["msg"] == "success"

        # put
        # 修改纸箱尺寸
        modify_data = {
            'length': 550,
            'width': 450,
            'height': 350,
            'weight': 10
        }
        res = client.put('http://127.0.0.1:7002/api/dpt/boxes/{}'.format(box_id), data=modify_data)
        res_data = res.json()
        assert res_data["msg"] == "success"
        box_data = res_data["data"]
        assert box_data["width"] == 450

        # get all
        # 查询所有纸箱
        res = client.get('http://127.0.0.1:7002/api/dpt/boxes/')
        res_data = res.json()
        assert res_data["msg"] == "success"
        box_data = res_data["data"]


        #get
        #通过过滤条件查询纸箱
        data =  {
                "length_min": "500",
                "length_max": "600"
                } 			
        res = client.get('http://127.0.0.1:7002/api/dpt/boxes/', data=data)
        res_data = res.json()
        assert res_data["msg"] == "success"


        # delete
        # 删除纸箱
        res = client.delete('http://127.0.0.1:7002/api/dpt/boxes/{}'.format(box_id))
        res_data = res.json()
        assert res_data["msg"] == "success"

        #get 
        # 根据id查询纸箱信息
        res = client.get('http://127.0.0.1:7002/api/dpt/boxes/{}'.format(box_id))
        res_data = res.json()
        assert res_data["code"] == -1
    """
    Test test_query_design.
    """
    def test_query_design(self):
        from utils.design import query_pallet_design_by_size, query_pallet_design_by_scan_code,query_pallet_id_by_size
        query_pallet_design_by_size(300,200,100,1)
        query_pallet_design_by_size(300,200,100)
        query_pallet_design_by_scan_code("123",1)
        query_pallet_design_by_scan_code("123")
        query_pallet_id_by_size(1200,1000,150)
        # 新增托盘,为了垛型规划
        delete_pallet = True
        # 查询是否已经有符合条件的托盘
        #get
        #通过过滤条件查询托盘
        data =  {
                "length_min": "1200",
                "length_max": "1200",
                "width_min": "1100",
                "width_max": "1100"
                }			
        res = client.get('http://127.0.0.1:7002/api/dpt/pallets/', data=data)
        res_data = res.json()
        assert res_data["msg"] == "success"

        records_total = res_data["data"]["records_total"]
        if records_total == 0:
            # post
            # 新增托盘
            data = {
				"name": "xyz",
				"length": 1200.0,
				"width": 1100.0,
				"height": 150.0,
				"max_height": 1200.0,
				"max_weight": 500.0,
				"pallet_type": False
	            }			
            res = client.post('http://127.0.0.1:7002/api/dpt/pallets/', data=data)
            res_data = res.json()
            assert res_data["msg"] == "success"
            pallet_data = res_data["data"]
            pallet_id = pallet_data["id"]
            delete_pallet = True
        else:
            pallet_data = res_data["data"]
            pallet_id = pallet_data["list"][0]["id"]
            delete_pallet = False
        #get 
        # 根据id查询托盘信息
        res = client.get('http://127.0.0.1:7002/api/dpt/pallets/{}'.format(pallet_id))
        res_data = res.json()
        assert res_data["msg"] == "success"


        # 新增纸箱，为了码垛规划
        delete_box = True
        # 查询是否已经有符合条件的箱子
        #get
        #通过过滤条件查询纸箱
        data =  {
                "length_min": "300",
                "length_max": "300",
                "width_min": "200",
                "width_max": "200",
                "height_min": "100",
                "height_max": "100"
                }			
        res = client.get('http://127.0.0.1:7002/api/dpt/boxes/', data=data)
        res_data = res.json()
        assert res_data["msg"] == "success"

        records_total = res_data["data"]["records_total"]
        if records_total == 0:
            # post
            # 新增纸箱
            data = {
                    "name": "box123",
                    "length": 300.0,
                    "width": 200.0,
                    "height": 100.0,
                    "weight": 12.0,
                    "scan_code": "xyz123"
                    }			
            res = client.post('http://127.0.0.1:7002/api/dpt/boxes/', data=data)
            res_data = res.json()
            assert res_data["msg"] == "success"
            box_data = res_data["data"]
            box_id = box_data["id"]
            delete_box = True
        else:
            box_data = res_data["data"]
            box_id = box_data["list"][0]["id"]
            delete_box = False


        # get
        # 查询纸箱
        res = client.get('http://127.0.0.1:7002/api/dpt/boxes/{}'.format(box_id))
        res_data = res.json()
        assert res_data["msg"] == "success"

        # post
        # 生成参考垛型
        data =  {
                "pallet_id": pallet_id,
                "box_id": box_id,
                "guillotine_packing": 1,
                "barcode_direction": 1,
                "mirror": 0,
                "flip": "",
                "layers": 5
                }			
        res = client.post('http://127.0.0.1:7002/api/dpt/design/', data=data)
        res_data = res.json()
        assert res_data["msg"] == "success", res_data

        layout_data = res_data["data"]
        # post
        # 新增垛型规划
        data = {
            "pallet_id": pallet_id,
            "box_id": box_id,
            "guillotine_packing": 1,
            "barcode_direction": 1,
            "mirror": 0,
            "flip": "",
            "layers": 5,
            "layout": layout_data
            }
        res = client.post('http://127.0.0.1:7002/api/dpt/design/', data=data)
        res_data = res.json()
        assert res_data["msg"] == "success"

        # get
        # 通过过滤条件查询垛型规划
        data =  {
                "length_min": "300",
                "length_max": "300",
                "width_min": "200",
                "width_max": "200",
                "height_min": "100",
                "height_max": "100"
                }		
        res = client.get('http://127.0.0.1:7002/api/dpt/design/', data=data)
        res_data = res.json()
        assert res_data["msg"] == "success"

        design_id = res_data["data"]["list"][0]["id"]

        # put
        # 修改垛型规划保存在数据库中的数据
        # 但是这个功能在HMI上其实是没有实际使用的，也不需要使用
        modify_data = {
                    "mirror": 1
                    }
        res = client.put('http://127.0.0.1:7002/api/dpt/design/{}'.format(design_id), data=modify_data)
        res_data = res.json()
        assert res_data["msg"] == "success"
        modify_data = {
                    "mirror": 0
                    }
        res = client.put('http://127.0.0.1:7002/api/dpt/design/{}'.format(design_id), data=modify_data)
        res_data = res.json()
        assert res_data["msg"] == "success"

        # get all
        # 查询所有垛型规划信息
        res = client.get('http://127.0.0.1:7002/api/dpt/design/')
        res_data = res.json()
        assert res_data["msg"] == "success"


        # XTF查询垛型规划数据
        data =  {
                "length": 300.0,
				"width": 200.0,
				"height": 100.0,
                "pallet_id":pallet_id
                }		
        res = client.get('http://127.0.0.1:7002/api/dpt/query_pallet_design/', data=data)
        res_data = res.json()
        assert res_data["msg"] == "success"

        test_pallet_id = query_pallet_id_by_size(1200,1100,150)
        assert test_pallet_id["id"] == pallet_id

        test_pallet_design =query_pallet_design_by_size(300,200,100,pallet_id)
        print(test_pallet_design)
        assert test_pallet_design["guillotine_packing"] == 1
        assert test_pallet_design["barcode_direction"] == 1
        assert test_pallet_design["mirror"] == 1
        assert test_pallet_design["layers"] == 5

        test_pallet_design =query_pallet_design_by_scan_code("xyz123",pallet_id)
        print(test_pallet_design)
        assert test_pallet_design["guillotine_packing"] == 1
        assert test_pallet_design["barcode_direction"] == 1
        assert test_pallet_design["mirror"] == 1
        assert test_pallet_design["layers"] == 5

        # delete
        # 删除垛型规划信息
        res = client.delete('http://127.0.0.1:7002/api/dpt/design/{}'.format(design_id))
        res_data = res.json()
        assert res_data["msg"] == "success"

        #get 
        # 根据id查询垛型规划信息
        res = client.get('http://127.0.0.1:7002/api/dpt/design/{}'.format(design_id))
        res_data = res.json()
        assert res_data["code"] == -1

        # 处理为了垛型规划而新增的纸箱
        if delete_box:
            # delete
            # 删除纸箱
            res = client.delete('http://127.0.0.1:7002/api/dpt/boxes/{}'.format(box_id))
            res_data = res.json()
            assert res_data["msg"] == "success"

            #get 
            # 根据id查询纸箱信息
            res = client.get('http://127.0.0.1:7002/api/dpt/boxes/{}'.format(box_id))
            res_data = res.json()
            assert res_data["code"] == -1
        
        # 处理为了垛型规划而新增的托盘
        if delete_pallet:
            # delete
            # 删除托盘
            res = client.delete('http://127.0.0.1:7002/api/dpt/pallets/{}'.format(pallet_id))
            res_data = res.json()
            assert res_data["msg"] == "success"

            #get 
            # 根据id查询托盘信息
            res = client.get('http://127.0.0.1:7002/api/dpt/pallets/{}'.format(pallet_id))
            res_data = res.json()
            assert res_data["code"] == -1

    """
    Test test_query_pallet_id_by_size.
    """
    def test_query_pallet_id_by_siez(self):
        from utils.design import query_pallet_id_by_size

        # post
        # 新增托盘1
        data = {
            "name": "p1",
            "length": 1200.0,
            "width": 1000.0,
            "height": 150.0,
            "max_height": 1200.0,
            "max_weight": 500.0,
            "pallet_type": False
            }			
        res = client.post('http://127.0.0.1:7002/api/dpt/pallets/', data=data)
        res_data = res.json()
        assert res_data["msg"] == "success"
        pallet_data = res_data["data"]
        pallet_id = pallet_data["id"]

        #get 
        # 根据id查询托盘信息
        res = client.get('http://127.0.0.1:7002/api/dpt/pallets/{}'.format(pallet_id))
        res_data = res.json()
        assert res_data["msg"] == "success"


        # post
        # 新增托盘1
        data = {
            "name": "p2",
            "length": 1200.0,
            "width": 1000.0,
            "height": 150.0,
            "max_height": 1200.0,
            "max_weight": 500.0,
            "pallet_type": False
            }			
        res = client.post('http://127.0.0.1:7002/api/dpt/pallets/', data=data)
        res_data = res.json()
        assert res_data["msg"] == "success"
        pallet_data = res_data["data"]
        pallet_id_2 = pallet_data["id"]

        #get 
        # 根据id查询托盘信息
        res = client.get('http://127.0.0.1:7002/api/dpt/pallets/{}'.format(pallet_id_2))
        res_data = res.json()
        assert res_data["msg"] == "success"

        test_pallet_id = query_pallet_id_by_size(1200,1000,150)
        assert test_pallet_id["code"] == -1

        # delete
        # 删除托盘
        res = client.delete('http://127.0.0.1:7002/api/dpt/pallets/{}'.format(pallet_id))
        res_data = res.json()
        assert res_data["msg"] == "success"

        #get 
        # 根据id查询托盘信息
        res = client.get('http://127.0.0.1:7002/api/dpt/pallets/{}'.format(pallet_id))
        res_data = res.json()
        assert res_data["code"] == -1

        # delete
        # 删除托盘
        res = client.delete('http://127.0.0.1:7002/api/dpt/pallets/{}'.format(pallet_id_2))
        res_data = res.json()
        assert res_data["msg"] == "success"

        #get 
        # 根据id查询托盘信息
        res = client.get('http://127.0.0.1:7002/api/dpt/pallets/{}'.format(pallet_id_2))
        res_data = res.json()
        assert res_data["code"] == -1


    """
    Test pallet_design query error.
    """
    def test_query_pallet_design_error(self):
        # 新增托盘,为了垛型规划
     
        # post
        # 新增托盘
        data = {
            "name": "x1",
            "length": 1200.0,
            "width": 1000.0,
            "height": 150.0,
            "max_height": 1200.0,
            "max_weight": 500.0,
            "pallet_type": False
            }			
        res = client.post('http://127.0.0.1:7002/api/dpt/pallets/', data=data)
        res_data = res.json()
        assert res_data["msg"] == "success"
        pallet_data = res_data["data"]
        pallet_id = pallet_data["id"]

        #get 
        # 根据id查询托盘信息
        res = client.get('http://127.0.0.1:7002/api/dpt/pallets/{}'.format(pallet_id))
        res_data = res.json()
        assert res_data["msg"] == "success"


        # 新增纸箱，为了码垛规划
        # post
        # 新增纸箱1
        data = {
                "name": "box1",
                "length": 300.0,
                "width": 200.0,
                "height": 100.0,
                "weight": 12.0,
                "scan_code": "box1"
                }			
        res = client.post('http://127.0.0.1:7002/api/dpt/boxes/', data=data)
        res_data = res.json()
        assert res_data["msg"] == "success"
        box_data = res_data["data"]
        box_id = box_data["id"]
   
        # get
        # 查询纸箱
        res = client.get('http://127.0.0.1:7002/api/dpt/boxes/{}'.format(box_id))
        res_data = res.json()
        assert res_data["msg"] == "success"

       # 新增纸箱2
        data = {
                "name": "box2",
                "length": 300.0,
                "width": 200.0,
                "height": 100.0,
                "weight": 12.0,
                "scan_code": "box2"
                }			
        res = client.post('http://127.0.0.1:7002/api/dpt/boxes/', data=data)
        res_data = res.json()
        assert res_data["msg"] == "success"
        box_data = res_data["data"]
        box_id_2 = box_data["id"]
   
        # get
        # 查询纸箱
        res = client.get('http://127.0.0.1:7002/api/dpt/boxes/{}'.format(box_id_2))
        res_data = res.json()
        assert res_data["msg"] == "success"


        # post
        # 生成参考垛型1
        data =  {
                "pallet_id": pallet_id,
                "box_id": box_id,
                "guillotine_packing": 1,
                "barcode_direction": 1,
                "mirror": 0,
                "flip": "",
                "layers": 5
                }			
        res = client.post('http://127.0.0.1:7002/api/dpt/design/', data=data)
        res_data = res.json()
        assert res_data["msg"] == "success"

        layout_data = res_data["data"]
        # post
        # 新增垛型规划
        data = {
            "pallet_id": pallet_id,
            "box_id": box_id,
            "guillotine_packing": 1,
            "barcode_direction": 1,
            "mirror": 0,
            "flip": "",
            "layers": 5,
            "layout": layout_data
            }
        res = client.post('http://127.0.0.1:7002/api/dpt/design/', data=data)
        res_data = res.json()
        assert res_data["msg"] == "success"



       # post
        # 生成参考垛型2
        data =  {
                "pallet_id": pallet_id,
                "box_id": box_id_2,
                "guillotine_packing": 1,
                "barcode_direction": 1,
                "mirror": 0,
                "flip": "",
                "layers": 5
                }			
        res = client.post('http://127.0.0.1:7002/api/dpt/design/', data=data)
        res_data = res.json()
        assert res_data["msg"] == "success"

        layout_data = res_data["data"]
        # post
        # 新增垛型规划
        data = {
            "pallet_id": pallet_id,
            "box_id": box_id_2,
            "guillotine_packing": 1,
            "barcode_direction": 1,
            "mirror": 0,
            "flip": "",
            "layers": 5,
            "layout": layout_data
            }
        res = client.post('http://127.0.0.1:7002/api/dpt/design/', data=data)
        res_data = res.json()
        assert res_data["msg"] == "success"


        # get
        # 通过过滤条件查询垛型规划
        data =  {
                "length_min": "300",
                "length_max": "300",
                "width_min": "200",
                "width_max": "200",
                "height_min": "100",
                "height_max": "100"
                }		
        res = client.get('http://127.0.0.1:7002/api/dpt/design/', data=data)
        res_data = res.json()
        assert res_data["msg"] == "success"

        design_id = res_data["data"]["list"][0]["id"]
        design_id_2 = res_data["data"]["list"][1]["id"]

       
        # XTF查询垛型规划数据
        data =  {
                "length": 300.0,
				"width": 200.0,
				"height": 100.0,
                "pallet_id":pallet_id
                }		
        res = client.get('http://127.0.0.1:7002/api/dpt/query_pallet_design/', data=data)
        res_data = res.json()
        assert res_data["code"] == -1

        # 通过查询垛型
        data =  {
				"scan_code": "box1",
                "pallet_id":pallet_id
                }		
        res = client.get('http://127.0.0.1:7002/api/dpt/query_pallet_design/', data=data)
        res_data = res.json()
        assert res_data["code"] == 0

        # 通过查询垛型
        data =  {
				"scan_code": "box2",
                "pallet_id":pallet_id
                }		
        res = client.get('http://127.0.0.1:7002/api/dpt/query_pallet_design/', data=data)
        res_data = res.json()
        assert res_data["code"] == 0

        


        # delete
        # 删除垛型规划信息
        res = client.delete('http://127.0.0.1:7002/api/dpt/design/{}'.format(design_id))
        res_data = res.json()
        assert res_data["msg"] == "success"

        #get 
        # 根据id查询垛型规划信息
        res = client.get('http://127.0.0.1:7002/api/dpt/design/{}'.format(design_id))
        res_data = res.json()
        assert res_data["code"] == -1

        # delete
        # 删除垛型规划信息
        res = client.delete('http://127.0.0.1:7002/api/dpt/design/{}'.format(design_id_2))
        res_data = res.json()
        assert res_data["msg"] == "success"

        #get 
        # 根据id查询垛型规划信息
        res = client.get('http://127.0.0.1:7002/api/dpt/design/{}'.format(design_id_2))
        res_data = res.json()
        assert res_data["code"] == -1

        # 处理为了垛型规划而新增的纸箱

        # delete
        # 删除纸箱
        res = client.delete('http://127.0.0.1:7002/api/dpt/boxes/{}'.format(box_id))
        res_data = res.json()
        assert res_data["msg"] == "success"

        #get 
        # 根据id查询纸箱信息
        res = client.get('http://127.0.0.1:7002/api/dpt/boxes/{}'.format(box_id))
        res_data = res.json()
        assert res_data["code"] == -1


        # delete
        # 删除纸箱
        res = client.delete('http://127.0.0.1:7002/api/dpt/boxes/{}'.format(box_id_2))
        res_data = res.json()
        assert res_data["msg"] == "success"

        #get 
        # 根据id查询纸箱信息
        res = client.get('http://127.0.0.1:7002/api/dpt/boxes/{}'.format(box_id_2))
        res_data = res.json()
        assert res_data["code"] == -1

        # delete
        # 删除托盘
        res = client.delete('http://127.0.0.1:7002/api/dpt/pallets/{}'.format(pallet_id))
        res_data = res.json()
        assert res_data["msg"] == "success"

        #get 
        # 根据id查询托盘信息
        res = client.get('http://127.0.0.1:7002/api/dpt/pallets/{}'.format(pallet_id))
        res_data = res.json()
        assert res_data["code"] == -1


if __name__ == "__main__":
    unittest.main()
