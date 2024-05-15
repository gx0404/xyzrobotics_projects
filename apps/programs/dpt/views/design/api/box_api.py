import sys
from collections import OrderedDict
from flask_restful import Resource, reqparse, fields, marshal
from werkzeug.datastructures import FileStorage
from apps.models import db
from apps.utils.upload_manager import check_upload_image, save_upload_file
from apps.helpers import make_json_response
from apps.programs.dpt.views.design.api.search_helper import SearchHelper, QueryType
from apps.programs.dpt.views.design.api.page_helper import PageHelper
from apps.programs.dpt.views.design.models import Box

# --- parser ---
box_parser = reqparse.RequestParser()
# box_parser.add_argument('id', type=int)
if sys.version_info.major == 2:
    box_parser.add_argument('name', type=unicode)
else:
    box_parser.add_argument('name', type=str)
box_parser.add_argument('scan_code', type=str)

# search_parser branch
search_parser = PageHelper.page_parser(box_parser)

box_parser.add_argument('length', type=float)
box_parser.add_argument('width', type=float)
box_parser.add_argument('height', type=float)
box_parser.add_argument('weight', type=float)
box_parser.add_argument('img_file', type=FileStorage, location='files')

# box_post_parser branch
box_post_parser = box_parser.copy()
# box_post_parser.replace_argument('name', type=unicode, required=True, help="No box name provided")

# search_parser
search_parser.add_argument('id', type=int)

search_parser.add_argument('length_min', type=float)
search_parser.add_argument('width_min', type=float)
search_parser.add_argument('height_min', type=float)
search_parser.add_argument('weight_min', type=float)

search_parser.add_argument('length_max', type=float)
search_parser.add_argument('width_max', type=float)
search_parser.add_argument('height_max', type=float)
search_parser.add_argument('weight_max', type=float)

# fields
box_fields = OrderedDict([
    ('id', fields.Integer),
    ('name', fields.String),
    ('length', fields.Float),
    ('width', fields.Float),
    ('height', fields.Float),
    ('weight', fields.Float),
    ('scan_code', fields.String),
    ('img_url', fields.String),
])


class BoxAPI(Resource):
    def get(self, _id):
        box = Box.query.get(_id)
        if not box or box.is_del != False:
            return make_json_response(code=-1, msg="No request box id")
        return make_json_response(data=marshal(box, box_fields))

    def delete(self, _id):
        box = Box.query.get(_id)
        if not box:
            return make_json_response(code=-1, msg="No request box id")
        db.session.delete(box)
        db.session.commit()
        return make_json_response()

    def put(self, _id):
        box = Box.query.get(_id)
        if not box:
            return make_json_response(code=-1, msg="No request box id")
        args = box_parser.parse_args(strict=True)
        for k, v in args.items():
            if v:
                box.__setattr__(k, v)
        db.session.commit()
        return make_json_response(data=marshal(box, box_fields))


class BoxListAPI(Resource):
    def get(self):
        args = search_parser.parse_args(strict=True)
        props = {
            'id': QueryType.NORMAL,
            'name': QueryType.FUZZY,
            'scan_code': QueryType.NORMAL,
            'length': QueryType.RANGE,
            'width': QueryType.RANGE,
            'height': QueryType.RANGE,
            'weight': QueryType.RANGE,
        }
        sh = SearchHelper(Box, search_parser)
        res = sh.filter_all(props, box_fields)
        return make_json_response(data=res)

    def post(self):
        args = box_post_parser.parse_args(strict=True)
        auto_name = False
        # check unique name
        if args['name']:
            unique_name = Box.query.filter_by(name=args['name']).first()
            if unique_name:
                return make_json_response(code=-1, msg='Name has been used')
        else:
            auto_name = True
            del args['name']
        if args['scan_code']:
            unique_box = Box.query.filter_by(scan_code=args['scan_code']).first()
            if unique_box:
                return make_json_response(code=-1, msg='scan_code has been used')

        # save image file
        content = args.get('img_file')
        del args['img_file']
        if content:
            # check image
            # print('Upload image:', content.__dict__)
            if not check_upload_image(content):
                return make_json_response(code=-1, msg='Image file type unsupported, support type:[jpg, png]')

        # save db info
        box = Box(**args)
        db.session.add(box)
        db.session.flush()
        if auto_name:
            box.name = u'box_{}'.format(box.id)
        if content:
            box.img_url = save_upload_file(content, filename='dpt_box_{}_{}'.format(box.id, content.filename))
        db.session.commit()
        return make_json_response(data=marshal(box, box_fields))
