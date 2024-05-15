"""
json2json - A JSON to JSON converter
json2json 是 piri 的一个子项目，用于将 JSON 转换为另一个 JSON

./piri 是 piri 的源代码，删除了其中 iso 相关的代码。
"""
import typing as t

from .build import build_tree

try:
    from .piri.process import process
except ImportError:
    raise ImportError("Please install `returns` at first by `pip install returns`")


def convert(data: t.Dict[str, t.Any], rule: t.Dict[str, str]) -> t.Dict[str, t.Any]:
    """转换

    Args:
        data (dict): 数据
        rule (dict): 映射规则

    Returns:
        dict: 转换后的数据
    """
    tree = build_tree(rule)
    if isinstance(data, dict):
        return process(data, tree)  # type: ignore
    else:
        raise TypeError("data must be dict")


def convert_raw(
    data: t.Dict[str, t.Any], config: t.Dict[str, t.Any]
) -> t.Union[list, dict]:
    """使用 piri 的配置文件进行转换

    Args:
        config (dict): 配置

    Returns:
        dict: 转换后的配置
    """
    return process(data, config)
