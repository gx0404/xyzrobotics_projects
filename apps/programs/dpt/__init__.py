from apps.programs.dpt.views.design.views import bp as design_bp
from apps.programs.dpt.views.helper.views import bp as helper_bp
from apps.programs.dpt.views.map_box.api import bp as map_box_bp
from apps.programs.dpt.views.palletize.views import bp as palletize_bp


threads = []
bp_list = [design_bp, helper_bp, palletize_bp, map_box_bp]
