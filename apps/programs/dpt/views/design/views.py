from flask import Blueprint
from flask_restful import Api

from apps.programs.dpt.views.design.api.box_api import BoxAPI, BoxListAPI
from apps.programs.dpt.views.design.api.pallet_api import PalletAPI, PalletListAPI
from apps.programs.dpt.views.design.api.design_api import DesignAPI, DesignListAPI, QueryPalletDesign

bp = Blueprint('dpt', __name__, url_prefix='/api/dpt')
api = Api(bp)


def add_restful_resource(name, url_path, base_api, list_api):
    api.add_resource(base_api, url_path + '<int:_id>', endpoint=name)
    api.add_resource(list_api, url_path)


add_restful_resource("pallet", "/pallets/", PalletAPI, PalletListAPI)
add_restful_resource("box", "/boxes/", BoxAPI, BoxListAPI)
add_restful_resource("design", "/design/", DesignAPI, DesignListAPI)
api.add_resource(QueryPalletDesign, "/query_pallet_design/")
