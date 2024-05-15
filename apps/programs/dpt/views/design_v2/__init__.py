from .api.box import bp as _box_bp
from .api.pallet import bp as _pallet_bp
from .api.plan import bp as _plan_bp

design_bps = [_box_bp, _plan_bp, _pallet_bp]
