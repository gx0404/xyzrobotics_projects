"""
提供system相关的API.
"""
import base64

from flask import request
from flask.blueprints import Blueprint

from apps import settings, openapi
from apps.utils.i18n import change_locale
from apps.utils.validation import validator
from apps.schemas import Response
from apps.base_app.flask_hook import req_log
from apps.base_app.views.system.entity import LogoEntity, SystemEntity
from apps.base_app.views.system.schemas import LogoResponse, SystemInfoResponse, UpdateLanguageSchema

system_bp = Blueprint("system", __name__, url_prefix="/api/system")


@system_bp.route("/", methods=["GET"])
@req_log(ignoreRes=True)
@validator()
@openapi.api_doc(tags=["XLHB(标准库接口汇总)", "系统信息"])
def get_system_info():
    """获取系统信息.

    Returns:
        ---
        {
            "code": 0,
            "message": "success",
            "data": {
                "version": "1.5.0",
                "language": 1,
                "logo": "xxx"
            }
        }
        ---
    """
    system = SystemEntity(version=settings.VERSION,
                          language=settings.LANGUAGE,
                          logo=settings.LOGO)
    return SystemInfoResponse(data=system)


@system_bp.route("/logo/", methods=["POST"])
@req_log(ignoreRes=True)
@validator()
@openapi.api_doc(tags=["XLHB(标准库接口汇总)", "系统信息"])
def edit_logo():
    """更新logo."""
    file = request.files["file"]
    settings.LOGO_FILENAME = "logo." + file.filename.rsplit(".", 1)[-1]
    byte = file._file.read()
    settings.LOGO = base64.b64encode(byte).decode()
    settings.dumps()
    settings.save_logo(byte)
    return LogoResponse(data=LogoEntity(logo=settings.LOGO))


@system_bp.route("/logo/", methods=["GET"])
@req_log(ignoreRes=True)
@validator()
@openapi.api_doc(tags=["XLHB(标准库接口汇总)", "系统信息"])
def get_logo():
    """获取logo图片."""
    entity = LogoEntity(logo=settings.LOGO)
    return LogoResponse(data=entity)


@system_bp.route("/lang/", methods=["POST"])
@req_log()
@validator()
@openapi.api_doc(tags=["XLHB(标准库接口汇总)", "系统信息"])
def edit_language(body: UpdateLanguageSchema):
    """更改系统语言."""
    settings.LANGUAGE = body.language
    change_locale()
    settings.dumps()
    return Response()
