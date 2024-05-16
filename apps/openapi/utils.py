#  -*- coding: utf-8 -*-
#  Copyright (c) XYZ Robotics Inc. - All Rights Reserved
#  Unauthorized copying of this file, via any medium is strictly prohibited
#  Proprietary and confidential
#  Author: Yuhang Wu <yuhang.wu@xyzrobotics.ai>, 2022/11/16 上午8:31
import re
from typing import Iterator, Optional, Tuple, Type

from pydantic import BaseConfig, create_model as _create_model, BaseModel
from marshmallow import Schema

from apps.utils.pydantic_from_marshmallow import (
    pydantic_from_marshmallow as _pydantic_from_marshmallow,
)


RE_FLASK_RULE = re.compile(
    r"""
    (?P<static>[^<]*)                           # static rule data
    <
    (?:
        (?P<converter>[a-zA-Z_][a-zA-Z0-9_]*)   # converter name
        (?:\((?P<args>.*?)\))?                  # converter arguments
        \:                                      # variable delimiter
    )?
    (?P<variable>[a-zA-Z_][a-zA-Z0-9_]*)        # variable name
    >
    """,
    re.VERBOSE,
)


def werkzeug_parse_rule(
    rule: str,
) -> Iterator[Tuple[Optional[str], Optional[str], str]]:
    """A copy of werkzeug.parse_rule which is now removed.

    Parse a rule and return it as generator. Each iteration yields tuples
    in the form ``(converter, arguments, variable)``. If the converter is
    `None` it's a static url part, otherwise it's a dynamic one.
    """
    pos = 0
    end = len(rule)
    do_match = RE_FLASK_RULE.match
    used_names = set()
    while pos < end:
        m = do_match(rule, pos)
        if m is None:
            break
        data = m.groupdict()
        if data["static"]:
            yield None, None, data["static"]
        variable = data["variable"]
        converter = data["converter"] or "default"
        if variable in used_names:
            raise ValueError(f"variable name {variable!r} used twice.")
        used_names.add(variable)
        yield converter, data["args"] or None, variable
        pos = m.end()
    if pos < end:
        remaining = rule[pos:]
        if ">" in remaining or "<" in remaining:
            raise ValueError(f"malformed url rule: {rule!r}")
        yield None, None, remaining


def create_model(name: str, example: dict) -> Type[BaseModel]:
    """Create a model."""

    class Config(BaseConfig):
        schema_extra = {"example": example}

    return _create_model(name, __config__=Config, **example)


def pydantic_from_marshmallow(schema: Type[Schema]) -> Type[BaseModel]:
    """Create a pydantic model from marshmallow schema."""
    model = _pydantic_from_marshmallow(schema)
    if hasattr(schema, "Config"):
        config = schema.Config

        class Config(BaseConfig):
            schema_extra = hasattr(config, "schema_extra") and config.schema_extra

        model.__config__ = Config
    return model
