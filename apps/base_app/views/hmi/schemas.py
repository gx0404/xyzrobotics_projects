from email.policy import default
from xmlrpc.client import boolean
from marshmallow import Schema, fields, ValidationError, validate

#Validate the yaml format

"""
{ 
  "ui": True,
  "url_prefix": "/api/custom/",
  "content": [
    {
      "func_type": 1,
      "route": "example_but",
      "button": {
        "label": "Example Button"
      },
      "helper": "Write the helper mesage here",  #Helper could be None
    },
    {
      "func_type": 2,
      "route": "example_but_with_input",
      "button": {
        "label": "check"
      },
      "input": {
        "label": "Example Button with Input",
        "key": "num",
        "input_type": "int",
        "default_value": 123
      },
      "helper": "Write the helper mesage here",   
    }

  ]
}
"""


class DefaultValue(fields.Field):
    def _deserialize(self, value, attr, data, **kwargs):
        if type(value) not in [int, bool, str]:
            raise ValidationError("{}'s type is not in [int, bool, double]".format(value))
        else:
            return value

class ButtonSchema(Schema):
    label = fields.String(required=True)

class InputSchema(Schema):
    label = fields.String(required=True)
    key = fields.String(required=True)
    input_type = fields.String(required=True, validate=validate.OneOf(["int", "bool", "double", "string"]))
    default_value = DefaultValue(required=False)
            

class HmiContentSchema(Schema):
    func_type = fields.Integer(required=True)
    route = fields.String(required=True)
    button = fields.Nested(ButtonSchema, required=True)
    input = fields.Nested(InputSchema, required=False)
    helper = fields.String(required=True)

class CustomizedHmiSchema(Schema):
    ui = fields.Boolean(required=True)
    url_prefix = fields.String(required=True)
    content = fields.List(fields.Nested(HmiContentSchema), required=True)