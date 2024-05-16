from collections import OrderedDict
from flask_restful import Resource, reqparse, fields, marshal, inputs

import sys
from apps.models import db
from apps.helpers import make_json_response
from .search_helper import SearchHelper, QueryType
from .page_helper import PageHelper
from ..models import Pallet


# --- parser ---

pallet_parser = reqparse.RequestParser()
# pallet_parser.add_argument('id', type=int)
if sys.version_info.major == 2:
    pallet_parser.add_argument('name', type=unicode)
else:
    pallet_parser.add_argument('name', type=str)
pallet_parser.add_argument('pallet_type', type=inputs.boolean)
# search_parser
search_parser = PageHelper.page_parser(pallet_parser)

pallet_parser.add_argument('length', type=float)
pallet_parser.add_argument('width', type=float)
pallet_parser.add_argument('height', type=float)
pallet_parser.add_argument('max_height', type=float)
pallet_parser.add_argument('max_weight', type=float)
pallet_parser.add_argument('thickness', type=float)

pallet_post_parser = pallet_parser.copy()
# pallet_post_parser.replace_argument('name', type=unicode, required=True, help="No pallet name provided")

# search_parser
search_parser.add_argument('id', type=int)

search_parser.add_argument('length_min', type=float)
search_parser.add_argument('width_min', type=float)
search_parser.add_argument('height_min', type=float)
search_parser.add_argument('max_height_min', type=float)
search_parser.add_argument('max_weight_min', type=float)
search_parser.add_argument('thickness_min', type=float)

search_parser.add_argument('length_max', type=float)
search_parser.add_argument('width_max', type=float)
search_parser.add_argument('height_max', type=float)
search_parser.add_argument('max_height_max', type=float)
search_parser.add_argument('max_weight_max', type=float)
search_parser.add_argument('thickness_max', type=float)

# fields
pallet_fields = OrderedDict([
    ('id', fields.Integer),
    ('name', fields.String),
    ('length', fields.Float),
    ('width', fields.Float),
    ('height', fields.Float),
    ('max_height', fields.Float),
    ('max_weight', fields.Float),
    ('pallet_type', fields.Boolean),
    ('thickness', fields.Float),
])


class PalletAPI(Resource):

    def get(self, _id):
        pallet = Pallet.query.get(_id)
        if not pallet or pallet.is_del != False:
            return make_json_response(code=-1, msg="No request pallet id")
        return make_json_response(data=marshal(pallet, pallet_fields))

    def delete(self, _id):
        pallet = Pallet.query.get(_id)
        if not pallet:
            return make_json_response(code=-1, msg="No request pallet id")
        db.session.delete(pallet)
        db.session.commit()
        return make_json_response()

    def put(self, _id):
        pallet = Pallet.query.get(_id)
        if not pallet:
            return make_json_response(code=-1, msg="No request pallet id")
        args = pallet_parser.parse_args(strict=True)
        for k, v in args.items():
            if v:
                pallet.__setattr__(k, v)
        db.session.commit()
        return make_json_response(data=marshal(pallet, pallet_fields))


class PalletListAPI(Resource):

    def get(self):
        args = search_parser.parse_args(strict=True)
        props = {
            'id': QueryType.NORMAL,
            'name': QueryType.FUZZY,
            'pallet_type': QueryType.NORMAL,
            'length': QueryType.RANGE,
            'width': QueryType.RANGE,
            'height': QueryType.RANGE,
            'max_height': QueryType.RANGE,
            'max_weight': QueryType.RANGE,
            'thickness': QueryType.RANGE,
        }
        sh = SearchHelper(Pallet, search_parser)
        res = sh.filter_all(props, pallet_fields)
        return make_json_response(data=res)

    def post(self):
        args = pallet_post_parser.parse_args(strict=True)
        auto_name = False
        # check unique name
        if args['name']:
            unique_name = Pallet.query.filter_by(name=args['name']).first()
            if unique_name:
                return make_json_response(code=-1, msg='Name has been used')
        else:
            auto_name = True
            del args['name']
        pallet = Pallet(**args)
        db.session.add(pallet)
        db.session.flush()
        if auto_name:
            pallet.name = u'pallet_{}'.format(pallet.id)
        db.session.commit()
        return make_json_response(data=marshal(pallet, pallet_fields))
