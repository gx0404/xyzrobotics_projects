import os
import logging
import logging.config

from apps import settings

COMMON_CONFIG = {
    "version": 1,
    "disable_existing_loggers": False,
    "formatters": {
        "standard": {
            "format": "[%(asctime)s %(levelname)7s][%(name)s] %(message)s [%(filename)s:%(lineno)d]"
        },
        "add_path": {
            "format": "[%(asctime)s %(levelname)s][%(name)s] %(message)s [%(pathname)s:%(lineno)d]"
        },
        "colored": {
            "()": "colorlog.ColoredFormatter",
            "format": "%(log_color)s[%(asctime)s %(levelname)7s][%(name)s]%(reset)s %(blue)s%(message)s%(reset)s%(log_color)s",
        },
    },
}


LOG_DEFAULT_CONF = {
    "handlers": {
        "console": {
            "level": "NOTSET",
            "formatter": "colored",
            "class": "logging.StreamHandler",
        },
        "web": {
            "level": logging.DEBUG if settings.DEBUG else logging.INFO,
            "formatter": "standard",
            "class": "apps.utils.logger.HMIRotatingFileHandler",
            "directory": settings.CMCT_LOG_PATH,
            "mode": "a",
            "maxBytes": 10485760,  # 10MB
            "backupCount": 5,
        },
        "hmi": {
            "level": "INFO",
            "formatter": "standard",
            "class": "apps.utils.logger.HMIRotatingFileHandler",
            "directory": os.path.join(settings.CMCT_LOG_PATH, "hmi_log"),
            "mode": "a",
            "maxBytes": 10485760,  # 10MB
            "backupCount": 2,
        },
        "wcs": {
            "level": "INFO",
            "formatter": "standard",
            "class": "apps.utils.logger.HMIRotatingFileHandler",
            "directory": os.path.join(settings.CMCT_LOG_PATH, "wcs_log"),
            "mode": "a",
            "maxBytes": 10485760,  # 10MB
            "backupCount": 2,
        },
        "plc": {
            "level": "INFO",
            "formatter": "standard",
            "class": "apps.utils.logger.HMIRotatingFileHandler",
            "directory": os.path.join(settings.CMCT_LOG_PATH, "plc_log"),
            "mode": "a",
            "maxBytes": 10485760,  # 10MB
            "backupCount": 2,
        },
        "mysql": {
            "level": "INFO",
            "formatter": "standard",
            "class": "apps.utils.logger.HMIRotatingFileHandler",
            "directory": os.path.join(settings.CMCT_LOG_PATH, "mysql_log"),
            "mode": "a",
            "maxBytes": 10485760,  # 10MB
            "backupCount": 2,
        },
        "customized": {
            "level": "DEBUG",
            "formatter": "standard",
            "class": "apps.utils.logger.HMIRotatingFileHandler",
            "directory": os.path.join(settings.CMCT_LOG_PATH, "customized_log"),
            "mode": "a",
            "maxBytes": 10485760,  # 10MB
            "backupCount": 2,
        },
        "errorlog": {
            "level": "ERROR",
            "formatter": "standard",
            "class": "logging.handlers.RotatingFileHandler",
            "filename": os.path.join(settings.CMCT_LOG_PATH, "error.log"),
            "mode": "a",
            "maxBytes": 2097152,  # 2MB
            "backupCount": 5,
        },
    },
    "loggers": {
        "apps": {
            "handlers": ["console", "web", "errorlog"],
            "level": logging.DEBUG if settings.DEBUG else logging.INFO,
            "propagate": True,
        },
        "werkzeug": {
            "handlers": ["console", "web", "errorlog"],
            "level": logging.DEBUG if settings.DEBUG else logging.INFO,
            "propagate": True,
        },
        "WCS": {
            "handlers": ["console", "web", "wcs", "errorlog"],
            "level": logging.DEBUG if settings.DEBUG else logging.INFO,
            "propagate": True,
        },
        "PLC": {
            "handlers": ["console", "web", "plc", "errorlog"],
            "level": logging.DEBUG if settings.DEBUG else logging.INFO,
            "propagate": True,
        },
        "HMI": {
            "handlers": ["console", "web", "hmi", "errorlog"],
            "level": logging.DEBUG if settings.DEBUG else logging.INFO,
            "propagate": True,
        },
        "SYSTEM": {
            "handlers": ["console", "web", "customized", "errorlog"],
            "level": logging.DEBUG if settings.DEBUG else logging.INFO,
            "propagate": True,
        },
        "alembic": {
            "handlers": ["console", "web", "errorlog"],
            "level": logging.DEBUG if settings.DEBUG else logging.INFO,
            "propagate": True,
        },
        "sqlalchemy": {
            "handlers": ["console", "mysql", "errorlog"],
            "level": logging.DEBUG if settings.DEBUG else logging.ERROR,
            "propagate": True,
        },
        "flask_migrate": {
            "handlers": ["console", "web", "errorlog"],
            "level": logging.DEBUG if settings.DEBUG else logging.INFO,
            "propagate": True,
        },
        "root": {
            "handlers": ["console", "web", "errorlog"],
            "level": logging.DEBUG if settings.DEBUG else logging.WARN,
            "propagate": True,
        },
    },
    **COMMON_CONFIG,
}


def get_logger(name, log_path=None):
    """获取logger.

    Args:
        name: logger名称
        log_path: 日志文件路径

    Returns:
        logger: logger对象
    """

    logger_config = {
        "handlers": {
            "console": {
                "level": "NOTSET",
                "formatter": "colored",
                "class": "logging.StreamHandler",
            },
            "web": {
                "level": logging.DEBUG if settings.DEBUG else logging.INFO,
                "formatter": "standard",
                "class": "apps.utils.logger.HMIRotatingFileHandler",
                "directory": settings.CMCT_LOG_PATH,
                "mode": "a",
                "maxBytes": 10485760,  # 10MB
                "backupCount": 5,
            },
            "errorlog": {
                "level": "ERROR",
                "formatter": "standard",
                "class": "logging.handlers.RotatingFileHandler",
                "filename": os.path.join(settings.CMCT_LOG_PATH, "error.log"),
                "mode": "a",
                "maxBytes": 2097152,  # 2MB
                "backupCount": 5,
            },
            name: {
                "level": "INFO",
                "formatter": "standard",
                "class": "apps.utils.logger.HMIRotatingFileHandler",
                "directory": log_path
                or os.path.join(settings.CMCT_LOG_PATH, name.lower() + "_log"),
                "mode": "a",
                "maxBytes": 10485760,  # 10MB
                "backupCount": 2,
            },
        },
        "loggers": {
            name: {
                "handlers": [name, "console", "web", "errorlog"],
                "level": logging.DEBUG if settings.DEBUG else logging.INFO,
                "propagate": True,
            }
        },
        **COMMON_CONFIG,
    }
    logging.config.dictConfig(logger_config)
    return logging.getLogger(name)


get_logger_obj = get_logger

hmi_log = outside_log = logging.getLogger("HMI")
wcs_log = logging.getLogger("WCS")
plc_log = logging.getLogger("PLC")
customized_log = logging.getLogger("SYSTEM")
xtf_log = get_logger("XTF")
