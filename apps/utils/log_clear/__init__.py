from .log_clear import ClearLog
from apps import settings

log_clear = ClearLog(settings.CMCT_LOG_PATH)
