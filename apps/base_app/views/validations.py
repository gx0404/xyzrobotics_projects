import sys
from marshmallow import Schema, fields

# command


class StartNodesSchema(Schema):
    node_id = fields.String()

    class Config:
        schema_extra = {"example": {"node_id": "xtf"}}


# notify


class NodeErrSchema(Schema):
    node_name = fields.String()
    timestamp = fields.Float()
    error_code = fields.Int()
    error_msg = fields.String()

    class Config:
        schema_extra = {
            "example": {
                "node_name": "xtf",
                "timestamp": 1669875812000,
                "error_code": 99999,
                "error_msg": "",
            }
        }


class SysLogSchema(Schema):
    msg_type = fields.String(required=True)  # 'error', 'info'
    code = fields.String(required=True)  # 'W0001', ...
    # If class is empty, seen as system

    class_ = fields.String(
        missing="sys", default="sys", dump_to="class", load_from="class"
    )
    object_ = fields.String(
        missing="", default="", dump_to="object", load_from="object"
    )

    tag = fields.String(missing="default", default="default")
    tool = fields.String(missing="", default="")
    worksapce = fields.String(missing="", default="")
    en_msg = fields.String(
        missing="", default=""
    )  # 'Dropped SKU', 'Nonconform SKU', ...
    zh_msg = fields.String(missing="", default="")
    en_tip = fields.String(missing="", default="")
    zh_tip = fields.String(missing="", default="")
    timeout = fields.Int(missing=None, default=None)


# query


class LogFilters(Schema):
    class_ = fields.List(fields.Str, dump_to="class", load_from="class")
    msg_type = fields.List(fields.Str)
    tag = fields.List(fields.Str)

    class Config:
        schema_extra = {"example": {"class": "", "msg_type": "info", "tag": "tag"}}


class QueryLogSchema(Schema):
    date = fields.String()
    # WARN: 由于 OpenAPI 在生成时是按成员名称生成的，如果使用 filters 将会导致无法找到对应的数据模型
    #  所以，这里使用使用 load_from, dump_to 设置别名.
    log_filters = fields.Nested(LogFilters, load_from="filters", dump_to="filters")


    class Config:
        schema_extra = {
            "example": {
                "date": "2022-01-01",
                "filters": {"class": "", "msg_type": "info", "tag": "tag"},
            }
        }


class QueryLogMenuSchema(Schema):
    date = fields.String()

    class Config:
        schema_extra = {"example": {"date": "2022-01-01"}}


class BackLogSchema(Schema):
    date = fields.String()
    check = fields.Boolean(requried=False)

    class Config:
        schema_extra = {"example": {"date": "2022-01-01", "check": True}}


class NodeLogSchema(Schema):
    node_name = fields.String(required=True)
    num_of_log = fields.Int()

    class Config:
        schema_extra = {"example": {"node_name": "0-robot_node", "num_of_log": 1000}}


class DownloadNodeLogSchema(Schema):
    node_name = fields.String(required=True)

    class Config:
        schema_extra = {"example": {"node_name": "xtf"}}
