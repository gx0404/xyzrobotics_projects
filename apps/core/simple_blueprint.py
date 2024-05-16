#  -*- coding: utf-8 -*-
#  Copyright (c) XYZ Robotics Inc. - All Rights Reserved
#  Unauthorized copying of this file, via any medium is strictly prohibited
#  Proprietary and confidential
#  Author: Yuhang Wu <yuhang.wu@xyzrobotics.ai>, 2022/8/9 下午2:08
from logging import Logger
from typing import Iterable, Callable

from flask.blueprints import Blueprint, _sentinel

from apps.log import hmi_log
from apps.utils.validation import validator
from apps.base_app.flask_hook import req_log


class SimpleBlueprint(Blueprint):
    """简单蓝图类

    提供更为简单的方式定义蓝图及视图函数，支持数据验证，日志记录，请求方式装饰器

    Examples:
        from apps.log import wcs_log
        from apps import SimpleBlueprint

        sbp = SimpleBlueprint("wcs", __name__, logger=wcs_log)

        @sbp.post("/test/")
        def main(body: BodySchema):
            pass

        equivalent to:

        from flask import Blueprint
        from apps import catch_log

        bp = Blueprint("wcs", __name__, url_prefix="/api/wcs/")

        @bp.route("/test/", methods=["POST"])
        @catch_log(wcs_log)
        @validator()
        def main(body: BodySchema):
            pass
    """

    def __init__(
        self,
        name,
        import_name,
        static_folder=None,
        static_url_path=None,
        template_folder=None,
        url_prefix=None,
        subdomain=None,
        url_defaults=None,
        root_path=None,
        cli_group=_sentinel,
        logger: Logger = hmi_log,
        auto_register: bool = False,
        disable_validator: bool = False,
    ):
        """

        Args:
            name: 蓝图名称
            import_name: 导入名
            static_folder: 静态文件目录
            static_url_path: 静态文件url路径
            template_folder: 模板文件目录
            url_prefix: url前缀
            subdomain:
            url_defaults:
            root_path: 根目录
            cli_group: 脚手架分组
            logger: 日志记录对象
            auto_register: 是否自动注册到app上, 默认False, 暂时不可用
            disable_validator: 是否禁用参数验证器, 默认False.
        """
        self.logger = logger
        self.auto_register = auto_register
        self.disable_validator = disable_validator

        if url_prefix is None:
            url_prefix = f"/api/{name}"

        super().__init__(
            name,
            import_name,
            static_folder,
            static_url_path,
            template_folder,
            url_prefix,
            subdomain,
            url_defaults,
            root_path,
            cli_group,
        )

    def route(
        self,
        rule: str,
        methods: Iterable[str] = None,
        logger: Logger = None,
        ignore_req_logs: bool = False,
        ignore_res_logs: bool = False,
        disable_logger: bool = False,
        disable_validator: bool = False,
        validator_params: dict = None,
        **options
    ) -> Callable:
        """装饰器, 用于定义路由.

        Args:
            rule: 访问路径
            methods: 请求方式
            logger: 日志记录器
            ignore_req_logs: 是否忽略请求日志, 默认False.
            ignore_res_logs: 是否忽略响应日制, 默认True.
            validator_params: 用于验证器参数字典.
            disable_logger: 被装饰的视图函数禁用日志记录, 默认False.
            disable_validator: 被装饰的视图函数禁用参数验证, 默认False.
            **options:

        Returns:
            Callable: 被装饰的视图函数
        """
        options["methods"] = methods or ["GET"]
        if not disable_logger:
            options["logger"] = logger
            options["ignore_req_logs"] = ignore_req_logs
            options["ignore_res_logs"] = ignore_res_logs
        if not (disable_validator and self.disable_validator):
            if validator_params and isinstance(validator_params, dict):
                raise TypeError("validator_params must be a dictionary.")
            options["validator_params"] = validator_params or {}
        return super(SimpleBlueprint, self).route(rule, **options)

    def add_url_rule(self, rule, endpoint=None, view_func=None, **options):
        """Like :meth:`Flask.add_url_rule` but for a blueprint.  The endpoint for
        the :func:`url_for` function is prefixed with the name of the blueprint.
        """
        view_func = self.wrap_view_func(view_func, options)
        super(SimpleBlueprint, self).add_url_rule(rule, endpoint, view_func, **options)

    def wrap_view_func(self, view_func: Callable, options: dict) -> Callable:
        """包装视图函数.

        Args:
            view_func: 原始视图函数
            options: 参数字典

        Returns:
            Callable: 可调用对象.
        """
        # 加装日志记录
        logger = options.pop("logger", None) or self.logger
        ignore_req_logs = options.pop("ignore_req_logs", False)
        ignore_res_logs = options.pop("ignore_res_logs", False)
        view_func = req_log(ignore_req_logs, ignore_res_logs, log=logger)(
            view_func
        )

        # 数据验证装饰器
        validator_params = options.pop("validator_params", {})
        view_func = validator(**validator_params)(view_func)
        return view_func

    def get(
        self,
        rule: str,
        logger: Logger = None,
        ignore_req_logs: bool = False,
        ignore_res_logs: bool = False,
        disable_logger: bool = False,
        disable_validator: bool = False,
        validator_params: dict = None,
        **options
    ) -> Callable:
        """装饰器, 用于定义 get 请求路由.

        Args:
            rule: 访问路径
            logger: 日志记录器
            ignore_req_logs: 是否忽略请求日志, 默认False.
            ignore_res_logs: 是否忽略响应日制, 默认False.
            disable_logger: 被装饰的视图函数禁用日志记录, 默认False.
            disable_validator: 被装饰的视图函数禁用参数验证, 默认False.
            validator_params: 用于验证器参数字典.
            **options: 其他参数

        Returns:
            Callable: 被装饰的视图函数
        """
        return self.route(
            rule=rule,
            methods=["GET"],
            logger=logger,
            ignore_res_logs=ignore_res_logs,
            ignore_req_logs=ignore_req_logs,
            validator_params=validator_params,
            disable_logger=disable_logger,
            disable_validator=disable_validator,
            **options
        )

    def post(
        self,
        rule: str,
        logger: Logger = None,
        ignore_req_logs: bool = False,
        ignore_res_logs: bool = False,
        disable_logger: bool = False,
        disable_validator: bool = False,
        validator_params: dict = None,
        **options
    ) -> Callable:
        """装饰器, 用于定义 post 请求路由.

        Args:
            rule: 访问路径
            logger: 日志记录器
            ignore_req_logs: 是否忽略请求日志, 默认False.
            ignore_res_logs: 是否忽略响应日制, 默认False.
            disable_logger: 被装饰的视图函数禁用日志记录, 默认False.
            disable_validator: 被装饰的视图函数禁用参数验证, 默认False.
            validator_params: 用于验证器参数字典.
            **options: 其他参数

        Returns:
            Callable: 被装饰的视图函数
        """
        return self.route(
            rule=rule,
            methods=["POST"],
            logger=logger,
            ignore_res_logs=ignore_res_logs,
            ignore_req_logs=ignore_req_logs,
            validator_params=validator_params,
            disable_logger=disable_logger,
            disable_validator=disable_validator,
            **options
        )

    def patch(
        self,
        rule: str,
        logger: Logger = None,
        ignore_req_logs: bool = False,
        ignore_res_logs: bool = False,
        disable_logger: bool = False,
        disable_validator: bool = False,
        validator_params: dict = None,
        **options
    ) -> Callable:
        """装饰器, 用于定义 patch 请求路由.

        Args:
            rule: 访问路径
            logger: 日志记录器
            ignore_req_logs: 是否忽略请求日志, 默认False.
            ignore_res_logs: 是否忽略响应日制, 默认False.
            disable_logger: 被装饰的视图函数禁用日志记录, 默认False.
            disable_validator: 被装饰的视图函数禁用参数验证, 默认False.
            validator_params: 用于验证器参数字典.
            **options: 其他参数

        Returns:
            Callable: 被装饰的视图函数
        """
        return self.route(
            rule=rule,
            methods=["PATCH"],
            logger=logger,
            ignore_res_logs=ignore_res_logs,
            ignore_req_logs=ignore_req_logs,
            validator_params=validator_params,
            disable_logger=disable_logger,
            disable_validator=disable_validator,
            **options
        )

    def put(
        self,
        rule: str,
        logger: Logger = None,
        ignore_req_logs: bool = False,
        ignore_res_logs: bool = False,
        disable_logger: bool = False,
        disable_validator: bool = False,
        validator_params: dict = None,
        **options
    ) -> Callable:
        """装饰器, 用于定义 put 请求路由.

        Args:
            rule: 访问路径
            logger: 日志记录器
            ignore_req_logs: 是否忽略请求日志, 默认False.
            ignore_res_logs: 是否忽略响应日制, 默认False.
            disable_logger: 被装饰的视图函数禁用日志记录, 默认False.
            disable_validator: 被装饰的视图函数禁用参数验证, 默认False.
            validator_params: 用于验证器参数字典.
            **options: 其他参数

        Returns:
            Callable: 被装饰的视图函数
        """
        return self.route(
            rule=rule,
            methods=["PUT"],
            logger=logger,
            ignore_res_logs=ignore_res_logs,
            ignore_req_logs=ignore_req_logs,
            validator_params=validator_params,
            disable_logger=disable_logger,
            disable_validator=disable_validator,
            **options
        )

    def delete(
        self,
        rule: str,
        logger: Logger = None,
        ignore_req_logs: bool = False,
        ignore_res_logs: bool = False,
        disable_logger: bool = False,
        disable_validator: bool = False,
        validator_params: dict = None,
        **options
    ) -> Callable:
        """装饰器, 用于定义 delete 请求路由.

        Args:
            rule: 访问路径
            logger: 日志记录器
            ignore_req_logs: 是否忽略请求日志, 默认False.
            ignore_res_logs: 是否忽略响应日制, 默认False.
            disable_logger: 被装饰的视图函数禁用日志记录, 默认False.
            disable_validator: 被装饰的视图函数禁用参数验证, 默认False.
            validator_params: 用于验证器参数字典.
            **options: 其他参数

        Returns:
            Callable: 被装饰的视图函数
        """
        return self.route(
            rule=rule,
            methods=["DELETE"],
            logger=logger,
            ignore_res_logs=ignore_res_logs,
            ignore_req_logs=ignore_req_logs,
            validator_params=validator_params,
            disable_logger=disable_logger,
            disable_validator=disable_validator,
            **options
        )
