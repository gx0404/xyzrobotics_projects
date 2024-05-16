"""
用于处理异常的函数.
"""
from flask import Flask, Response
from werkzeug.exceptions import HTTPException

from apps.exceptions import XYZBaseError, XYZValidationError
from apps.helpers import make_json_response
from apps.log import hmi_log


def init_app(app: Flask):

    @app.errorhandler(Exception)
    def handle_generic_error(e: Exception) -> Response:
        """处理通用异常"""
        # 如果被此方法捕获, 则说明该异常未被主动捕获, 属于未知异常, 需要输出详细调用栈信息.
        hmi_log.error(repr(e), exc_info=True)
        return make_json_response(
            code=500,
            msg=f"Uncatch Exception: {e.__class__.__name__}",
            data={
                "error_code": 500,
                "error_message": str(e)
            }
        )

    @app.errorhandler(XYZBaseError)
    def handle_xyz_error(e: XYZBaseError) -> Response:
        """处理所有XYZBaseError异常."""
        hmi_log.error(e, exc_info=True)
        return make_json_response(
            code=e.code,
            msg=e.name,
            data={
                "error_code": e.error_code,
                "error_message": e.error_message
            }
        )

    @app.errorhandler(XYZValidationError)
    def handler_xyz_validation_error(e: XYZValidationError) -> Response:
        """处理数据格式验证失败的异常."""
        return make_json_response(
            code=400,
            msg=e.name,
            data={
                "error_code": e.error_code,
                "error_message": e.error_message
            }
        )

    @app.errorhandler(HTTPException)
    def handle_http_exception(e: HTTPException) -> Response:
        """处理http状态码属于异常码的异常."""
        if e.code == 500:
            hmi_log.error(msg=str(e), exc_info=True)
        return make_json_response(
            code=e.code,
            msg=e.name,
            data={
                "error_code": e.code,
                "error_message": e.description or str(e)
            }
        )
