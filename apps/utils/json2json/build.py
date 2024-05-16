"""
通过定义简单的映射规则，构建用于 piri 的转换规则树

Example:
    >>> build_tree("a.rootTasks[].taskNo", "b.tasks[].task_id")
"""
import copy
import typing as t


def build_attribute(source_keys: t.List[str], target_key: str) -> dict:
    """构建属性节点

    Example:
        >>> build_attribute("object.name", "name")
        {'name': 'name', 'mappings': [{'path': ['object', 'name']}]}

    Args:
        source_keys (str): 源数据的表达式
        target_key (str): 目标数据的键

    Returns:
        dict: 属性节点
    """
    path: t.List[str] = []
    source_keys = copy.copy(source_keys)
    source_keys.reverse()
    for key in source_keys:
        if key.endswith("[]"):
            path.append(key[:-2])
            break
        else:
            path.append(key)
    path.reverse()
    return {"name": target_key, "mappings": [{"path": path}]}


def build_array(
    source_keys: t.List[str], target_key: str, inner_object: bool, is_iterators: bool
) -> dict:
    """构建数组节点

    Example:
        >>> build_array("tasks", False)
        {'name': 'tasks', 'array': True, 'attributes': []}
        >>> build_array("tasks", True)
        {'name': 'tasks', 'array': True, 'objects': []}

    Args:
        target_key (str): 数组的键
        inner_object (bool): 数组内是否是对象
        is_iterators (bool): 是否是可迭代对象

    Returns:
        dict: 数组节点
    """
    if inner_object:
        node = {"name": target_key, "array": True, "objects": []}
    else:
        node = {"name": target_key, "array": True, "attributes": []}
    if is_iterators:
        node.update(build_iterables(source_keys))
    return node


def build_object(key) -> dict:
    """构建对象节点

    Example:
        >>> build_object("object")
        {'name': 'object', 'array': False, 'objects': []}

    Args:
        key (str): 键

    Returns:
        dict: 对象节点
    """
    return {"name": key, "array": False, "objects": []}


def build_iterables(source_keys: t.List[str]) -> dict:
    """构建可迭代对象

    Example:
        >>> build_iterables(["object", "tasks"])
        {'name': 'root', 'array': True, 'iterables': [{'alias': 'tasks', 'path': ['object', 'tasks']}], 'objects': [], 'attributes': []}

    Args:
        source_keys (str): 源数据的表达式

    Returns:
        list: 可迭代对象
    """
    path: t.List[str] = []
    for key in source_keys:
        if key.endswith("[]"):
            path.append(key[:-2])
            break
        else:
            path.append(key)
    return {"iterables": [{"alias": path[-1], "path": path}]}


def build_node(source_keys: t.List[str], target_keys: list, i: int) -> dict:
    """深度优先遍历, 自底向上构建树
        source_keys (str): 源数据的表达式
        target_keys (list): 目标数据的键
        i (int): 当前遍历到的层级

    a.rootTasks[].taskNo -> b.tasks[].task_id

    rootTasks[].taskNo -> a.b.tasks[].task_id

    当 source_keys 出现 `[]` 时，target_keys 肯定也会有与之对应的 `[]` 否则无法构建

    Returns:
        dict: 当前层级的节点
    """
    if i >= len(target_keys) - 1:
        return build_attribute(source_keys, target_keys[-1])
    child_node = build_node(source_keys, target_keys, i + 1)
    if target_keys[i].endswith("[]"):
        # array
        cur_node = build_array(
            source_keys,
            target_keys[i][:-2],
            is_object(child_node),
            is_iterators(source_keys),
        )
    else:
        # object
        cur_node = build_object(target_keys[i])

    if is_object(child_node):
        if "objects" not in cur_node:
            cur_node["objects"] = []
        cur_node["objects"].append(child_node)
    elif is_attribute(child_node):
        if "attributes" not in cur_node:
            cur_node["attributes"] = []
        cur_node["attributes"].append(child_node)
    elif is_array(child_node):
        if "objects" not in cur_node:
            cur_node["objects"] = []
        cur_node["objects"].append(child_node)
    else:
        raise ValueError("child node must be object or attribute")
    return cur_node


def build_tree(rule: t.Dict[str, str]) -> dict:
    """构建树

    Args:
        rule (dict): 映射规则

    Returns:
        dict: 树
    """
    root = {"name": "root", "array": False, "objects": [], "attributes": []}
    for source_expression, target_expression in rule.items():
        start_level, node = get_max_level(root, target_expression.split("."))
        child_node = build_node(
            source_expression.split("."), target_expression.split("."), start_level
        )
        if is_object(child_node) or is_array(child_node):
            if "objects" not in node:
                node["objects"] = []
            node["objects"].append(child_node)
        elif is_attribute(child_node):
            if "attributes" not in node:
                node["attributes"] = []
            node["attributes"].append(child_node)
    return root


def get_max_level(root: dict, keys: list) -> t.Tuple[int, dict]:
    """获取能到达的最深层级节点

    Args:
        root (dict): 根节点
        keys (list): 键列表

    Returns:
        int, dict: keys 能够到达的最大层级和最大层级的节点
    """
    level = 0
    for key in keys:
        if key.endswith("[]"):
            key = key[:-2]
        if "objects" not in root:
            break
        for obj in root["objects"]:
            if obj["name"] == key:
                root = obj
                level += 1
                break
    return level, root


def is_object(node: dict) -> bool:
    """判断节点是否是对象节点

    Args:
        node (dict): 节点

    Returns:
        bool: 是否是对象节点
    """
    if "array" in node and not node["array"]:
        return True
    return False


def is_array(node: dict) -> bool:
    """判断节点是否是数组节点

    Args:
        node (dict): 节点

    Returns:
        bool: 是否是数组节点
    """
    if "array" in node and node["array"]:
        return True
    return False


def is_attribute(node: dict) -> bool:
    """判断节点是否是属性节点

    Args:
        node (dict): 节点

    Returns:
        bool: 是否是属性节点
    """
    if "array" not in node and "objects" not in node:
        return True
    return False


# TODO: 优化, 加入缓存
def is_iterators(source_keys: t.List[str]) -> bool:
    """判断是否是可迭代对象

    Args:
        source_keys (str): 源数据的表达式

    Returns:
        bool: 是否是可迭代对象
    """
    for key in source_keys:
        if key.endswith("[]"):
            return True
    return False
