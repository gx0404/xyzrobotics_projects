from .settings import init_settings

wcs_settings = init_settings()

from . import app
from . import models
from . import error_handlers
from . import manager
from .app import init_app
from .exceptions import *
