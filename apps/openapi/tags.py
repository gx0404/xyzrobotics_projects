#  -*- coding: utf-8 -*-
#  Copyright (c) XYZ Robotics Inc. - All Rights Reserved
#  Unauthorized copying of this file, via any medium is strictly prohibited
#  Proprietary and confidential
#  Author: Yuhang Wu <yuhang.wu@xyzrobotics.ai>, 2022/11/15 下午1:47
from typing import Optional

import pydantic


class ExternalDocumentation(pydantic.BaseModel):
    """External documentation model for OpenAPI."""

    description: str = None
    url: str

    class Config:
        schema_extra = {
            "example": {
                "description": "Find out more about XYZ Robotics",
                "url": "https://xyzrobotics.ai",
            }
        }


class Tag(pydantic.BaseModel):
    """Tag model for OpenAPI."""
    name: str
    description: str = None
    externalDocs: Optional[ExternalDocumentation] = None

    class Config:
        frozen = True
        allow_mutation = False
        schema_extra = {
            "example": {
                "name": "Tag",
                "description": "Tag description",
                "externalDocs": {
                    "description": "Find out more",
                    "url": "https://xyzrobotics.ai",
                },
            }
        }

    def __hash__(self):
        return hash(self.name)


if __name__ == '__main__':
    print(Tag.schema_json(indent=2))
