# coding=utf-8
from __future__ import division

import json
from collections import OrderedDict

import numpy as np
import tf.transformations as tfm
from flask_restful import Resource, fields, marshal, reqparse
from sqlalchemy import text
from xyz_homogeneous_bin_packing import HomogeneousBinPacking as HBP
from xyz_homogeneous_bin_packing import NaiveStabilityChecker as StabilityChecker
from xyz_homogeneous_bin_packing import OrientedBinPacking as OBP

from apps.helpers import make_json_response
from apps.models import db
from apps.settings import settings
from apps.base_app.flask_hook import req_log
from .page_helper import PageHelper
from .search_helper import QueryType, SearchHelper
from ..models import Box, Design, Pallet
from ...design_v2.entity import Objects



class NumpyEncoder(json.JSONEncoder):
    def default(self, obj):
        if isinstance(obj, np.ndarray):
            return obj.tolist()
        return json.JSONEncoder.default(self, obj)


# --- parser ---
design_parser = reqparse.RequestParser()
# design_parser.add_argument('name', type=unicode)
# search_parser
search_parser = PageHelper.page_parser(design_parser)

design_parser.add_argument('pallet_id', type=int)
design_parser.add_argument('box_id', type=int)
design_parser.add_argument('guillotine_packing', type=int)
design_parser.add_argument('barcode_direction', type=int)
design_parser.add_argument('mirror', type=int)
design_parser.add_argument('flip', type=str)
design_parser.add_argument('layers', type=int)
design_parser.add_argument('layout', type=str)

# design_post_parser branch
design_post_parser = design_parser.copy()
# design_post_parser.replace_argument('name', type=unicode, required=True, help="No box name provided")

# search_parser
search_parser.add_argument('id', type=int)
search_parser.add_argument('box_name', type=str)
search_parser.add_argument('pallet_name', type=str)

search_parser.add_argument('length_min', type=float)
search_parser.add_argument('width_min', type=float)
search_parser.add_argument('height_min', type=float)
search_parser.add_argument('weight_min', type=float)

search_parser.add_argument('length_max', type=float)
search_parser.add_argument('width_max', type=float)
search_parser.add_argument('height_max', type=float)
search_parser.add_argument('weight_max', type=float)


# --- quaternions ---
def rotation_to_q(rotation):
    '''
    将HMI反馈的1，2，3，4，转换为四元数，然后给XTF调用使用
    https://xyz-robotics.atlassian.net/wiki/spaces/depalletizeproduct/pages/475562620/HMI+xlhb
    Args:
        rotation: int, 1,2,3,4数字
                1代表0度（纸箱不旋转）
                2代表90度（纸箱逆时针旋转90度）
                3代表180度（纸箱逆时针旋转180度）
                4代表-90度（纸箱顺时针旋转90度）
    Returns:
         四元数，qx,qy,qz,qw,表示旋转，list
    '''
    if rotation == 1:
        # 0
        return 0.0, 0.0, 0.0, 1.0  # x,y,z,w
    elif rotation == 2:
        # 90
        return 0.0, 0.0, 0.7071067811865475, 0.7071067811865476  # x,y,z,w
    elif rotation == 3:
        # 180
        return 0, 0, 1.0, 0.0  # x,y,z,w
    elif rotation == 4:
        # -90
        return 0.0, 0.0, -0.7071067811865475, 0.7071067811865476  # x,y,z,w
    else:
        raise Exception("垛型规划中参数输入错误")


def q_to_rotation(q):
    '''
    将pose中的四元数旋转转换为1，2，3，4，然后反馈给HMI端使用
    https://xyz-robotics.atlassian.net/wiki/spaces/depalletizeproduct/pages/475562620/HMI+xlhb
    Args:
        q: list,   x,y,z,qx,qy,qz,qw

    Returns:
        int, 1,2,3,4数字
                1代表0度（纸箱不旋转）
                2代表90度（纸箱逆时针旋转90度）
                3代表180度（纸箱逆时针旋转180度）
                4代表-90度（纸箱顺时针旋转90度）
    '''
    # FIXME(YuhangWu): xyz_motion库终止异常.
    #   当程序终止时，该库会抛出-6退出码，并抛出异常： double free or corruption (fasttop)
    #   详情见: https://xyz-robotics.atlassian.net/browse/MQWR-1115?atlOrigin=eyJpIjoiN2Q4ZGNlNWMyMTE0NDcxOThjZTQyMzBmYjc5ODdiNjgiLCJwIjoiaiJ9
    from xyz_motion import SE3

    vp = SE3(q)
    vquat = tfm.quaternion_from_matrix(vp.homogeneous)
    box_r = tfm.euler_from_quaternion(vquat)
    box_rotation = np.rad2deg(box_r)
    box_angle = np.float(box_rotation[2])
    if box_angle > -2.0 and box_angle < 2.0:
        return 1
    elif box_angle > 88.0 and box_angle < 92.0:
        return 2        
    elif box_angle > 178.0 and box_angle < 182.0:
        return 3
    elif box_angle > -92.0 and box_angle < -88.0:
        return 4
    else:
        raise Exception("垛型规划中参数输入错误")

def layout_expand(layouts):
    """ transform layout json data , mm -> m """
    # x,y,rotation -> x,y,z,qx,qy,qz,qw,id
    return [np.array([[box[0], box[1], 0] + list(rotation_to_q(box[2])) for box in layout]) for layout in layouts]


def layout_shrink(layouts):
    #  x,y,z,qx,qy,qz,qw,id ->  x,y,rotation , m -> mm
    return [
        [
            [box[0], box[1], q_to_rotation(box[0:7])] for box in layout
        ] for layout in layouts
    ]


# fields
design_fields = OrderedDict([
    ('id', fields.Integer),
    # ('name', fields.String),
    ('pallet_id', fields.Integer),
    ('box_id', fields.Integer),
    ('box_name', fields.String(attribute='box.name')),
    ('box_length', fields.String(attribute='box.length')),
    ('box_width', fields.String(attribute='box.width')),
    ('box_height', fields.String(attribute='box.height')),
    ('box_weight', fields.String(attribute='box.weight')),
    ('pallet_name', fields.String(attribute='pallet.name')),
    ('layout', fields.String),
])


class DesignAPI(Resource):

    def get(self, _id):
        design = Design.query.get(_id)
        if not design:
            return make_json_response(code=-1, msg="No request design id")
        return make_json_response(data=marshal(design, design_fields))

    def delete(self, _id):
        design = Design.query.get(_id)
        if not design:
            return make_json_response(code=-1, msg="No request design id")
        db.session.delete(design)
        db.session.commit()
        return make_json_response()

    def put(self, _id):
        design = Design.query.get(_id)
        if not design:
            return make_json_response(code=-1, msg="No request design id")
        args = design_parser.parse_args(strict=True)
        for k, v in args.items():
            if v:
                design.__setattr__(k, v)
        db.session.commit()
        return make_json_response(data=marshal(design, design_fields))


class DesignListAPI(Resource):

    def get(self):
        """ Search """
        args = search_parser.parse_args(strict=True)
        prop_design = {
            'id': QueryType.NORMAL,
        }
        prop_box = {
            'length': QueryType.RANGE,
            'width': QueryType.RANGE,
            'height': QueryType.RANGE,
            'weight': QueryType.RANGE,
        }

        query_set = Design.query
        design_sh = SearchHelper(Design, search_parser)

        query_set = design_sh.filter(prop_design, query_set)
        query_set = query_set.join(Box)
        query_set = query_set.join(Pallet)

        if args['pallet_name']:
            query_set = query_set.filter(Pallet.name.like(u"%{}%".format(args['pallet_name'])))
        if args['box_name']:
            query_set = query_set.filter(Box.name.like(u"%{}%".format(args['box_name'])))

        box_sh = SearchHelper(Box, search_parser)
        query_set = box_sh.filter(prop_box, query_set)
        query_set = query_set.order_by(text("-design.id"))
        print(query_set)
        res = box_sh.query_set_wrapper(query_set, design_fields)
        return make_json_response(data=res)

    def post(self):
        """ Upload """
        args = design_parser.parse_args(strict=True)

        pallet = Pallet.query.get(args['pallet_id'])
        if not pallet:
            return make_json_response(code=-1, msg='Pallet id incorrect')
        box = Box.query.get(args['box_id'])
        if not box:
            return make_json_response(code=-1, msg='Box id incorrect')

        # check unique design
        design = Design.query.filter_by(pallet_id=pallet.id, box_id=box.id, is_del=False).first()
        if design is not None:
            return make_json_response(code=-1, msg='Design with this box and pallet already exists')

        layout = args['layout']
        l, w, h = box.length, box.width, box.height
        L, W, H = pallet.length, pallet.width, pallet.max_height
        l, w, h, L, W, H = map(float, [l, w, h, L, W, H])

        layers = args['layers']

        if not layout:
            # request suggest plan
            # 调用自动生成拆码垛方案

            # barcode_direction
            # 0： 无条码
            # 1： 条码在两个长边，朝外
            # 2： 条码在一个长边，朝外
            # 3： 条码在两个短边，朝外
            # 4： 条码在一个短边，朝外
            barcode_direction = args['barcode_direction']

            # guillotine_packing：0是非回型垛，1是回型垛
            guillotine_packing = args['guillotine_packing']

            mirror = args['mirror']
            flip = args['flip']
            # print('barcode_direction：{}，guillotine_packing：{}，mirror：{}，flip：{}'.format(barcode_direction,guillotine_packing,mirror,flip))

            if barcode_direction == 0:
                # 无条码朝向要求，自由垛型
                solver = HBP([l, w, h], [L, W, H], spacing = False)
            elif barcode_direction in [1, 2]:
                # 条码在长边
                solver = OBP([l, w, h], [L, W, H], spacing = False, guillotine_packing = guillotine_packing, long_edge_outside = True)
            elif barcode_direction in [3, 4]:
                # 条码在短边
                solver = OBP([l, w, h], [L, W, H], spacing = False, guillotine_packing = guillotine_packing, long_edge_outside = False)
            
            n = solver.solve()
            if solver.n_layers == 0 or n <= 0:
                return make_json_response(code=-1, msg="No box pose can be returned. Please check the pallet dimensions and the box dimensions")

            odd_poses, even_poses = solver.generate_both_layouts(mirror)
        
            if flip != '':
                odd_poses = np.array(odd_poses)
                even_poses = np.array(even_poses)
                if flip in ['x', 'X']:
                    if barcode_direction == 0:
                        odd_poses = odd_poses * [-1, 1, 1, 1, 1, 1, 1]
                        even_poses = even_poses * [-1, 1, 1, 1, 1, 1, 1]
                    else:
                        odd_poses, even_poses = solver.flip(flip)
                elif flip in ['y', 'Y']:
                    if barcode_direction == 0:
                        odd_poses = odd_poses * [1, -1, 1, 1, 1, 1, 1]
                        even_poses = even_poses * [1, -1, 1, 1, 1, 1, 1]
                    else:
                        odd_poses, even_poses = solver.flip(flip)
                else:
                    return make_json_response(code=-1, msg="error flip parameter")
           
            layouts = []
            layouts.append(odd_poses)
            layouts.append(even_poses)
            layouts = layout_shrink(layouts)
            data = json.dumps(layouts, cls=NumpyEncoder)
            return make_json_response(code=0, msg="success", data=data)
        else:
            # TODO 后端Layout有效性检查
            if isinstance(layout, str):
                layout = json.loads(layout)
            layout = layout_expand(layout)
            if len(layout) == 1:
                layer_0 = layer_1 = layout[0]
            elif len(layout) == 2:
                layer_0, layer_1 = layout[0], layout[1]
            else:
                return make_json_response(code=-1, msg="layout format error")
            try:
                sc = StabilityChecker(layer_0, layer_1, [l, w, h], [L, W, H])
                passed = sc.stability_check()
            except IndexError:
                print("垛型规划稳定性检查不通过")
                passed = False
            if passed:
                save_design(args)
                return make_json_response(code=0, msg="success")
            else:
                return make_json_response(code=-1, msg="failure")


def save_design(args):
    design = Design(pallet_id=args['pallet_id'], box_id=args['box_id'], 
                    guillotine_packing=args['guillotine_packing'],
                    barcode_direction=args['barcode_direction'], 
                    mirror=args['mirror'], flip=args['flip'], layers=args['layers'],
                    layout=args['layout'])
    db.session.add(design)
    db.session.commit()


query_parser = reqparse.RequestParser()

query_parser.add_argument('pallet_id', type=int)
query_parser.add_argument('scan_code', type=str)
query_parser.add_argument('length', type=float)
query_parser.add_argument('width', type=float)
query_parser.add_argument('height', type=float)


class QueryPalletDesign(Resource):
    @req_log()
    def get(self):
        """Query pallet design in HMI backend by pallet id and sku id.

        Args:
            pallet_id: int,非必须参数，如果给pallet_id，则按照给的pallet_id进行搜索
                        如果不给pallet_id，则按照后端config文件设置中的DEFAULT_PALLET_ID进行托盘检索。
            
            scan_code: str
            or
            length: Float，box length
            width: Float，box width
            height: Float，box height

        Returns:
            code: int,0: 正常   -1: 异常
            msg： str,讯息 
            data: dict,垛型规划数据
        """
        # box = Box.query.get(sku_id)
        # pallet = Pallet.query.get(pallet_id)
        args = query_parser.parse_args()
        arg_pallet_id = args.get('pallet_id')
        if arg_pallet_id is None:
            pallet_id = settings.DEFAULT_PALLET_ID
        else:
            pallet_id = arg_pallet_id

        scan_code = args.get('scan_code')
        if scan_code is None:
            length = args.get('length')
            width = args.get('width')
            height = args.get('height')
            box_query = Box.query.filter_by(length=length, width=width, height=height, is_del=False)
            box_count = box_query.count()
            if box_count > 1:
                return make_json_response(code=-1, msg="query the count of this box's pallet_design is more than one.")
            box = box_query.first()
            if box:
                box_id = box.id
                design = (
                    Design.query.join(Pallet)
                    .join(Box)
                    .filter(
                        Design.pallet_id == pallet_id,
                        Box.id == box_id,
                        Design.is_del == False,
                    )
                    .first()
                )
            else:
                return make_json_response(code=-1, msg="No request design")
        else:
            design_query = (
                Design.query.join(Pallet)
                .join(Box)
                .filter(
                    Design.pallet_id == pallet_id,
                    Box.scan_code == scan_code,
                    Design.is_del == False,
                )
            )
            design_count = design_query.count()
            if design_count > 1:
                return make_json_response(code=-1, msg="query the count of this box's pallet_design is more than one.")
            design = design_query.first()

        # l, w, h = box.length, box.width, box.height
        # L, W, H = pallet.length, pallet.width, pallet.height
        # l, w, h, L, W, H = map(float, [l, w, h, L, W, H])
        data = []
        if design:
            layout = design.layout
            if isinstance(layout, str):
                layout = json.loads(layout)
            elif layout is None:
                objects = Objects.parse_obj(design.objects)
                layout = objects.to_layout()
                layout = json.loads(layout.json())

            layout = layout_expand(layout)
            layout_data = json.loads(json.dumps(layout, cls=NumpyEncoder))

            output_data = {
                "guillotine_packing": design.guillotine_packing,
                "barcode_direction": design.barcode_direction,
                "mirror": design.mirror,
                "flip": design.flip,
                "layers": design.layers,
                "layout": layout_data,
                "objects": design.objects
            }
            return make_json_response(data=output_data)
        else:
            return make_json_response(code=-1, msg="No request design")


# class CurrentDesignGetter(Resource):
#     def get(self):
#         layer_0 = layer_1 = ''
#         l, w, h, L, W, H = [0] * 6
#         if 'box_id' in session and 'pallet_id' in session:
#             box_id = session['box_id']
#             pallet_id = session['pallet_id']
#             box = Box.query.get(box_id)
#             pallet = Pallet.query.get(pallet_id)
#             design = Design.query.filter_by(pallet_id=pallet_id, box_id=box_id).first()
#
#             l, w, h = box.length, box.width, box.height
#             L, W, H = pallet.length, pallet.width, pallet.height
#             l, w, h, L, W, H = map(float, [l, w, h, L, W, H])
#
#             layout = json.loads(design.layout)
#             layout = layout_expand(layout)
#
#             if len(layout) == 1:
#                 layer_0 = layer_1 = layout[0]
#             elif len(layout) == 2:
#                 layer_0, layer_1 = layout[0], layout[1]
#
#         elif 'sku_info' in session and 'pallet_id' in session:
#             l, w, h = session['sku_info']
#             pallet_id = session['pallet_id']
#             pallet = Pallet.query.get(pallet_id)
#             L, W, H = pallet.length, pallet.width, pallet.height
#             solver = HBP([l, w, h], [L, W, H])
#             solver.solve()
#             # layouts = solver.visualize_solutions(visualize=False)
#             layouts = solver.generate_both_layouts(0)
#             print(layouts)
#             layer_0, layer_1 = layouts[0], layouts[1]
#
#         data = {
#             'layouts': [layer_0, layer_1],
#             'box_info': [l, w, h],
#             'pallet_info': [L, W, H]
#         }
#         data = json.dumps(data, cls=NumpyEncoder)
#         return data
