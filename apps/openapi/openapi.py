#  -*- coding: utf-8 -*-
#  Copyright (c) XYZ Robotics Inc. - All Rights Reserved
#  Unauthorized copying of this file, via any medium is strictly prohibited
#  Proprietary and confidential
#  Author: Yuhang Wu <yuhang.wu@xyzrobotics.ai>, 2022/11/15 下午1:27
import inspect
import json
import typing as t
from typing import List, Optional, Sequence, Callable

from marshmallow import Schema
from pydantic import BaseModel
from werkzeug.routing import parse_converter_args

from apps.utils import util
from .page import DEFAULT_PAGE_TEMPLATES
from .responses import JSONResponse, Response
from .schemas.response import STDResponseSchema
from .tags import Tag
from .utils import create_model, werkzeug_parse_rule, pydantic_from_marshmallow

if t.TYPE_CHECKING:
    from flask import Flask


class OpenAPI:
    def __init__(
        self,
        app: t.Optional["Flask"] = None,
        host: t.Optional[str] = None,
        title: t.Optional[str] = None,
        version: t.Optional[str] = None,
        description: t.Optional[str] = None,
    ):
        """Initialize OpenAPI.

        Args:
            host (str): Host for the API.
            title (str): Title for the API.
            version (str): Version for the API.
            description (str): Description for the API.
        """
        self.app = app
        self.blueprint_state = None
        self.spec = {}
        self._registered = False
        self._paths = {}
        self._tags: t.Set[Tag] = set()
        self._host = host
        self._title = title
        self._version = version
        self._description = description
        self._components: t.Dict[str, t.Dict] = {
            "schemas": {},
            "securitySchemes": {},
            "parameters": {},
            "requestBodies": {},
            "responses": {},
            "headers": {},
            "examples": {},
            "links": {},
            "callbacks": {},
        }

    def init_app(
        self,
        app: "Flask",
        host: str,
        title: str,
        version: str,
        description: str = "",
    ):
        """Initialize OpenAPI.

        Args:
            app (Flask): Flask app.
            host (str): Host for the API.
            title (str): Title for the API.
            version (str): Version for the API.
            description (str): Description for the API.
        """
        self.app = app
        self._host = host
        self._title = title
        self._version = version
        self._description = description
        not self._registered and self.register_route(app)

    def api_doc(
        self,
        tags: Sequence[str],
        summary: Optional[str] = None,
        description: Optional[str] = None,
        deprecated: bool = False,
        operation_id: Optional[str] = None,
        security=None,
        request_body: t.Union[t.Type[BaseModel], t.Type[Schema], None] = None,
        response_body: t.Union[t.Type[BaseModel], t.Type[Schema], None] = None,
        response=None,
        parameters=None,
        **kwargs,
    ):
        """Decorator for OpenAPI.

        Args:
            tags (List[str]): Tags for the API.
            summary (str): Summary for the API.
            description (Optional[str], optional): Description for the API. Defaults to None. if None, function docstring will be used.
            deprecated (bool, optional): Whether the API is deprecated. Defaults to False.
            operation_id (str, optional): Operation ID for the API. Defaults to None. if None, function name will be used.
            security ([type], optional): Security for the API. Defaults to None.
            request_body ([type], optional): Request body for the API. Defaults to None.
            response_body ([type], optional): Response body for the API. Defaults to None.
            response ([type], optional): Response for the API. Defaults to None.
            parameters ([type], optional): Parameters for the API. Defaults to None.
        """
        for tag in tags:
            self.add_tag(tag)

        def decorator(func):
            func.__apidoc__ = {
                "tags": tags,
                "summary": summary,
                "description": description,
                "deprecated": deprecated,
                "operation_id": operation_id,
                "security": security,
                "request_body": request_body,
                "response_body": response_body,
                "response": response,
                "parameters": parameters,
                **kwargs,
            }
            return func

        return decorator

    def add_tag(self, name: str, description: str = None, externalDocs: dict = None):
        """Add a tag to the API.

        Args:
            name (str): Name of the tag.
            description (str, optional): Description of the tag. Defaults to None.
            externalDocs (dict, optional): External documentation of the tag. Defaults to None.
        """
        self._tags.add(
            Tag(name=name, description=description, externalDocs=externalDocs)
        )

    def add_schema(self, schema: t.Type[BaseModel]):
        """Add a schema.

        Args:
            schema (Type[BaseModel]): Schema.
        """
        if (
            not issubclass(schema, BaseModel)
            or schema.__name__ in self._components["schemas"]
        ):
            return
        # 递归 schema，添加其中所有为 BaseModel 类型的 schema
        for field in schema.__fields__.values():
            if inspect.isclass(field) and issubclass(field.type_, BaseModel):
                self.add_schema(field.type_)
        self._components["schemas"][schema.__name__] = schema

    def register_route(self, app: "Flask"):
        """Register OpenAPI to the app.

        Args:
            app (Flask): Flask app.
        """
        app.add_url_rule(
            rule="/openapi.json",
            endpoint="/openapi_apidoc",
            view_func=lambda: json.dumps(self.get_openapi()),
        )
        app.add_url_rule(
            rule="/wcs_adaptor_openapi.json",
            endpoint="/wcs_adaptor/openapi_apidoc",
            view_func=lambda: json.dumps(self.get_openapi_only_wcs_adaptor()),
        )
        for ui in DEFAULT_PAGE_TEMPLATES:
            app.add_url_rule(
                rule=f"/apidoc/{ui}/",
                endpoint=f"openapi_apidoc_{ui}",
                view_func=lambda ui=ui: DEFAULT_PAGE_TEMPLATES[ui].format(
                    spec_url="/openapi.json",
                    spec_path="apidoc",
                    client_id="",
                    client_secret="",
                    realm="",
                    app_name="XLH API",
                    scope_separator=" ",
                    scopes="[]",
                    additional_query_string_params="{}",
                    use_basic_authentication_with_access_code_grant="false",
                    use_pkce_with_authorization_code_grant="false",
                ),
            )
            app.add_url_rule(
                rule=f"/apidoc/wcs_adaptor/{ui}",
                endpoint=f"wcs_adaptor_openapi_apidoc_{ui}",
                view_func=lambda ui=ui: DEFAULT_PAGE_TEMPLATES[ui].format(
                    spec_url="/wcs_adaptor_openapi.json",
                    spec_path="apidoc",
                    client_id="",
                    client_secret="",
                    realm="",
                    app_name="WCS ADAPTOR API",
                    scope_separator=" ",
                    scopes="[]",
                    additional_query_string_params="{}",
                    use_basic_authentication_with_access_code_grant="false",
                    use_pkce_with_authorization_code_grant="false",
                ),
            )
        self._registered = True

    def bypass(self, method: str) -> bool:
        """Whether to bypass the method."""
        return method in ("HEAD", "OPTIONS")

    def parse_path(
        self,
        route: Optional[t.Mapping[str, str]],
        path_parameter_descriptions: Optional[t.Mapping[str, str]],
    ) -> t.Tuple[str, list]:
        subs = []
        parameters = []

        for converter, arguments, variable in werkzeug_parse_rule(str(route)):
            if converter is None:
                subs.append(variable)
                continue
            subs.append(f"{{{variable}}}")

            args: tuple = ()
            kwargs: dict = {}

            if arguments:
                args, kwargs = parse_converter_args(arguments)

            schema = None
            if converter == "any":
                schema = {
                    "type": "string",
                    "enum": args,
                }
            elif converter == "int":
                schema = {
                    "type": "integer",
                    "format": "int32",
                }
                if "max" in kwargs:
                    schema["maximum"] = kwargs["max"]
                if "min" in kwargs:
                    schema["minimum"] = kwargs["min"]
            elif converter == "float":
                schema = {
                    "type": "number",
                    "format": "float",
                }
            elif converter == "uuid":
                schema = {
                    "type": "string",
                    "format": "uuid",
                }
            elif converter == "path":
                schema = {
                    "type": "string",
                    "format": "path",
                }
            elif converter == "string":
                schema = {
                    "type": "string",
                }
                for prop in ["length", "maxLength", "minLength"]:
                    if prop in kwargs:
                        schema[prop] = kwargs[prop]
            elif converter == "default":
                schema = {"type": "string"}

            description = (
                path_parameter_descriptions.get(variable, "")
                if path_parameter_descriptions
                else ""
            )
            parameters.append(
                {
                    "name": variable,
                    "in": "path",
                    "required": True,
                    "schema": schema,
                    "description": description,
                }
            )

        return "".join(subs), parameters

    def parse_request(self, func: t.Callable) -> t.Optional[dict]:
        """Parse request body."""

        def _make(model: t.Type[BaseModel]) -> dict:
            self.add_schema(model)
            return {
                "content": {
                    "application/json": {
                        "schema": {"$ref": f"#/components/schemas/{model.__name__}"}
                    }
                }
            }

        request_body = func.__apidoc__["request_body"]
        if request_body:
            if isinstance(request_body, dict):
                # 将请求体转为为 BaseModel，方便下文统一处理
                request_body = create_model(
                    f"{func.__name__}_RequestModel", request_body
                )

            if issubclass(request_body, BaseModel):
                return _make(request_body)
            elif issubclass(request_body, Schema):
                return _make(pydantic_from_marshmallow(request_body))
            else:
                raise TypeError("request_body must be dict or BaseModel")
        else:
            body, _ = self.parse_body_and_returns_from_docstring(func)
            if body:
                request_body = create_model(f"{func.__name__}_RequestModel", body)
                return _make(request_body)

    def parse_response(self, func: t.Callable) -> dict:
        """Parse response body."""
        response: t.Optional[Response] = func.__apidoc__["response"]
        if response is None:
            if response_body := func.__apidoc__["response_body"]:
                response = JSONResponse(model=response_body)
            else:
                # 从函数的文档字符串中解析响应

                # TODO: parse_body_and_returns_from_docstring 被调用了两次
                #   可以整合到一个函数中
                _, returns = self.parse_body_and_returns_from_docstring(func)
                if returns:
                    response = JSONResponse(
                        model={f"{self.parse_name(func)}ResponseModel": returns}
                    )
                else:
                    # 如果没有文档字符串，则使用通用的响应体
                    response = JSONResponse(model=STDResponseSchema)
        hasattr(response, "model") and self.add_schema(response.model)
        # TODO(YuhangWU): 状态码暂时只有 200
        return {
            "200": response.dict(exclude_none=True, exclude={"model"}),
        }

    def parse_params(
        self,
        func: t.Callable[..., t.Any],
        params: List[t.Mapping[str, t.Any]],
        models: t.Mapping[str, t.Any],
    ) -> List[t.Mapping[str, t.Any]]:
        """get spec for (query, headers, cookies)"""
        attr_to_spec_key = {"query": "query", "headers": "header", "cookies": "cookie"}
        route_param_keywords = ("explode", "style", "allowReserved")

        for attr in attr_to_spec_key:
            if hasattr(func, attr):
                model = models[getattr(func, attr)]
                properties = model.get("properties", {model.get("title"): model})
                for name, schema in properties.items():
                    # Route parameters keywords taken out of schema level
                    extra = {
                        kw: schema.pop(kw)
                        for kw in route_param_keywords
                        if kw in schema
                    }
                    t = {
                        "name": name,
                        "in": attr_to_spec_key[attr],
                        "schema": schema,
                        "description": schema.get("description", ""),
                        **extra,
                    }
                    if "required" in model:
                        t["required"] = model.get("required")
                    params.append(t)

        return params

    def parse_comments(self, func: t.Callable) -> tuple:
        """Parse comments.

        Args:
            func (function): Function.

        Returns:
            tuple: Summary and description.
        """
        summary = ""
        desc = ""
        if func.__doc__:
            doc = func.__doc__.split("\n")
            summary = doc[0]
            desc = "\n".join(doc[1:])
        return summary, desc

    def parse_body_and_returns_from_docstring(self, func: t.Callable) -> tuple:
        """Parse comments."""
        docstring = inspect.getdoc(func)
        body = {}
        returns = {}

        def to_dict(s: str) -> dict:
            try:
                return ast.literal_eval(textwrap.dedent(s).strip())
            except ValueError:
                raise SyntaxError(f"func: {func.__name__}, 转为字典失败, 请检查文档字符串是否符合规范")

        if docstring:
            import ast
            import textwrap

            lines = docstring.split("---")
            i = 0
            while i < len(lines):
                try:
                    if "Body:" in lines[i]:
                        # 默认下一个就是 body
                        i += 1
                        body = to_dict(lines[i])
                    elif "Returns:" in lines[i]:
                        # 默认下一个就是返回值
                        i += 1
                        returns = to_dict(lines[i])
                    i += 1
                    if body and returns:
                        break
                except (IndexError, SyntaxError) as e:
                    raise RuntimeError(
                        f"func: {func.__name__}, 文档字符串解析失败, 请检查文档字符串是否符合规范"
                    ) from e
        return body, returns

    def parse_func(self, route: t.Any):
        if self.blueprint_state:
            func = self.blueprint_state.app.view_functions[route.endpoint]
        else:
            func = self.app.view_functions[route.endpoint]

        # view class: https://flask.palletsprojects.com/en/1.1.x/views/
        if getattr(func, "view_class", None):
            cls = getattr(func, "view_class")
            for method in route.methods:
                view = getattr(cls, method.lower(), None)
                if view:
                    yield method, view
        else:
            for method in route.methods:
                yield method, func

    def parse_name(self, func: Callable) -> str:
        # TODO: 使用缓存，存储已解析过的函数名
        return func.__qualname__

    def make_tags(self):
        """Make tags."""
        self._tags = [tag.dict(exclude_none=True) for tag in self._tags]

    def make_paths(self):
        """Make paths."""
        for rule in self.app.url_map.iter_rules():
            for method, func in self.parse_func(rule):
                if self.bypass(method):
                    continue
                if rule.endpoint in ("static", "openapi"):
                    continue
                if not hasattr(func, "__apidoc__"):
                    continue
                path_parameter_descriptions = getattr(
                    func, "path_parameter_descriptions", None
                )
                path, parameters = self.parse_path(rule, path_parameter_descriptions)
                summary, desc = self.parse_comments(func)
                name = self.parse_name(func)
                if path not in self._paths:
                    self._paths[path] = {}
                if method not in self._paths[path]:
                    operationId = f"{name}_{path.replace('/', '_')}" + "_" + method

                    self._paths[path][method.lower()] = {
                        "operationId": operationId,
                        "tags": func.__apidoc__["tags"],
                        "summary": func.__apidoc__["summary"]
                        or summary
                        or f"{name} <{method}>",
                        "deprecated": func.__apidoc__["deprecated"],
                        "description": desc,
                        "parameters": self.parse_params(func, parameters[:], {}),
                    }
                    try:
                        self._paths[path][method.lower()][
                            "responses"
                        ] = self.parse_response(func)
                    except Exception as e:
                        raise RuntimeError(f"func: {name}, 响应解析失败") from e

                    if method.lower() == "get":
                        continue
                    request_body = self.parse_request(func)
                    if request_body:
                        self._paths[path][method.lower()]["requestBody"] = request_body

    def make_components(self):
        """Make components."""
        self._components["schemas"].update(self.make_schemas()["schemas"])

    def make_schemas(self) -> dict:
        """Make schemas."""
        schemas: t.Dict[str, t.Type[BaseModel]] = self._components["schemas"]
        results = {"schemas": {}}
        for key, model in schemas.items():
            ret = model.schema()
            new = {
                "description": ret.get("description", ""),
                "type": ret["type"],
                "properties": ret["properties"],
            }
            if "required" in ret:
                new["required"] = ret.get("required")
            if "example" in ret:
                new["example"] = ret.get("example")
            definitions = ret.get("definitions", {})
            for k, v in definitions.items():
                results["schemas"][k] = v
            # if definitions:
            # for prop_name, prop_val in ret["properties"].items():
            # {"$ref": "#/definitions/Code"} -> {"$ref": "#/components/schemas/Code"}
            # if "$ref" in prop_val:
            #     new["properties"][prop_name] = {
            #         "$ref": "#/components/schemas/{}".format(
            #             util.to_pascal_case(prop_name)
            #         )
            #     }
            results["schemas"][key] = new
        return json.loads(
            json.dumps(results)
            .replace("definitions", "components/schemas")
            .replace("exclusiveMinimum", "minimum")
            .replace("exclusiveMaximum", "maximum")
        )

    def get_openapi(self) -> dict:
        """Get OpenAPI specification.

        Returns:
            dict: OpenAPI specification.
        """
        if self.app is None:
            raise RuntimeError("Flask application is not initialized.")
        if self.spec:
            return self.spec
        self.make_tags()
        self.make_paths()
        self.make_components()
        self.spec = {
            "openapi": "3.0.0",
            "info": {
                "title": self._title,
                "version": self._version,
                "description": self._description,
            },
            "servers": [
                {
                    "url": f"http://{self._host}",
                    "description": "XLHB API",
                }
            ],
            "tags": self._tags,
            "paths": self._paths,
            "components": self._components,
        }
        return self.spec

    def get_openapi_only_wcs_adaptor(self) -> dict:
        """仅返回 WCS Adaptor 相关的 API"""
        if self.app is None:
            raise RuntimeError("Flask application is not initialized.")
        if not self.spec:
            self.get_openapi()

        tags = []
        wcs_adaptor_paths = {}
        for path, methods in self._paths.items():
            for method, data in methods.items():
                if "WCS" in data["tags"] or "XTF" in data["tags"]:
                    wcs_adaptor_paths[path] = methods
                    tags.extend(data["tags"])
                    break

        tags = [{"name": tag} for tag in set(tags)]

        return {
            "openapi": "3.0.0",
            "info": {
                "title": self._title,
                "version": self._version,
                "description": self._description,
            },
            "servers": [
                {
                    "url": f"http://{self._host}",
                    "description": "WCS ADAPTOR API",
                }
            ],
            "tags": tags,
            "paths": wcs_adaptor_paths,
            "components": self._components,
        }
