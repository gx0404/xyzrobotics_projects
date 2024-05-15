# -*- coding: utf-8 -*-
"""
Copyright (c) XYZ Robotics Inc. - All Rights Reserved
Unauthorized copying of this file, via any medium is strictly prohibited
Proprietary and confidential
Author: Yuhang Wu <yuhang.wu@xyzrobotics.ai>, 2022-07-26
"""
import json
import pathlib
from typing import List, Literal, Optional, Union, Dict

from pydantic import BaseModel, Extra, Field, root_validator, validator

from apps import settings
from apps.exceptions import XYZValidationError
from .enums import BarcodeDirection, Flip, Mirror, ClockwiseRotation


class BoxEntity(BaseModel):
    id: int
    name: str = Field(title="名称")
    length: float = Field(title="长(mm)", gt=0, le=20000)
    width: float = Field(title="宽(mm)", gt=0, le=20000)
    height: float = Field(title="高(mm)", gt=0, le=20000)
    weight: float = Field(title="重量(kg)", gt=0, le=10000)
    scan_code: Optional[str] = Field(default=None, title="条码")
    img_url: Optional[str] = None
    is_del: Optional[bool] = False

    class Config:
        orm_mode = True


class PalletEntity(BaseModel):
    id: int
    name: str = Field(title="托盘名称")
    length: float = Field(title="长", gt=0, le=20000)
    width: float = Field(title="宽", gt=0, le=20000)
    height: float = Field(title="高", gt=0, le=20000)
    max_height: float = Field(title="最大高度", gt=0, le=20000)
    max_weight: float = Field(title="最大再重量", gt=0, le=10000)
    # TODO(YuhangWu): 之后版本更改其类型, 不应该使用布尔类型
    pallet_type: bool  # True: 深框 False: 托盘
    thickness: Optional[float] = Field(title="壁厚", gt=0, le=1000)
    is_del: Optional[bool] = False

    # class NumpyEncoder(json.JSONEncoder):
    #     def default(self, obj):
    #         if isinstance(obj, np.ndarray):
    #             return obj.tolist()
    #         return json.JSONEncoder.default(self, obj)

    class Config:
        orm_mode = True
        json_encoders = {}


class Layout(BaseModel):
    # 有两个list，第一个表示奇数层，第二个表示偶数层
    # 内部有三个元素，分别表示 (x坐标，y坐标，旋转方式)
    # layout: Tuple[Tuple[float, float, ClockwiseRotation], ...]
    __root__: List[List]


class Object(BaseModel):
    """物体属性"""

    label: int = Field(ge=1, title="数字标签")
    x: float = Field(title="x坐标")
    y: float = Field(title="y坐标")
    rotation: ClockwiseRotation = Field(title="旋转方式")
    place_drop_buffer: float = Field(default=0, ge=0, title="下限高度")


class Objects(BaseModel):
    """物体奇偶层布局"""

    odd_layer: List[Object] = Field(title="奇数层")
    even_layer: List[Object] = Field(default_factory=list, title="偶数层, 允许为一个空列表")

    @property
    def num_per_layer(self) -> int:
        """每层数量"""
        return len(self.odd_layer)

    @root_validator()
    def root_validate(cls, values: Dict[str, List[Object]]) -> Dict[str, List[Object]]:
        """每一个 Object 的 label 不能重复.

        Args:
            values: 待校验数据.
        """

        def _validate(v):
            s = {item.label for item in v}
            if len(s) != len(v):
                raise XYZValidationError(error_message="Label 属性校验失败，请检查 Label 属性是否重复.")

        for _, _v in values.items():
            _validate(_v)
        return values

    def to_layout(self) -> Layout:
        """将 objects 转为旧版的 layout 格式."""
        # 按 label 升序排列
        odd_layer = sorted(self.odd_layer, key=lambda x: x.label)
        even_layer = sorted(self.even_layer, key=lambda x: x.label)
        return Layout(
            __root__=[
                [[l.x, l.y, l.rotation] for l in odd_layer],
                [[l.x, l.y, l.rotation] for l in even_layer],
            ]
        )

    @classmethod
    def from_layout(cls, obj: Union[Layout, List, str]) -> "Objects":
        """根据 ``layout`` 转为 ``Objects``."""
        if isinstance(obj, str):
            obj = Layout(__root__=json.loads(obj))
        elif isinstance(obj, list):
            obj = Layout(__root__=obj)
        odd, even = obj.__root__

        def _make_layer(items: List[List]) -> List[Object]:
            layer = []
            for inx, item in enumerate(items):
                layer.append(
                    Object(
                        label=inx + 1,  # label 下标从 1 开始
                        x=item[0],
                        y=item[1],
                        rotation=item[2],
                    )
                )
            return layer

        return cls(odd_layer=_make_layer(odd), even_layer=_make_layer(even))


class PlanEntity(BaseModel):
    """规划实体

    .. versionchanged:: v1.4.0
        修改 ``layout`` 数据来源于 ``objects``.
    """

    id: int
    box: BoxEntity
    pallet: PalletEntity
    guillotine_packing: Literal[0, 1]
    barcode_direction: BarcodeDirection
    mirror: Mirror
    flip: Flip
    layout: Union[Layout, List[List], str, None]
    layers: int = Field(ge=0, title="层数")
    num_per_layer: int = Field(default=0, ge=0, title="每层箱数")
    num_per_pallet: int = Field(default=0, ge=0, title="每托箱数")
    image_url: Optional[str] = Field(title="垛型图片链接")
    objects: Optional[Objects] = Field(default=None, title="物体奇偶层布局")
    is_del: Optional[bool] = False

    @property
    def image_real_path(self) -> pathlib.Path:
        """返回图片的真实地址."""
        # /static/images/1.png -> /images/1.png
        image_url = self.image_url.replace("/static/", "", 1)
        return pathlib.Path(settings.STATIC_FOLDER, image_url)

    @validator("layout")
    def validate_layout(cls, v: Union[Layout, List[List], str]) -> Layout:
        if isinstance(v, list):
            v = Layout(__root__=v)
        elif isinstance(v, str):
            v = Layout(__root__=json.loads(v))
        return v

    @root_validator()
    def root_validate(cls, values: dict) -> dict:
        """校验数据."""
        if "layout" not in values or "objects" not in values:
            return values
        layout = values.get("layout")
        objects: Objects = values.get("objects", None)
        if layout is None and objects:
            values["layout"] = objects.to_layout()
        if objects is None and layout:
            objects = Objects.from_layout(layout)
            values["objects"] = objects
        values["num_per_layer"] = objects.num_per_layer
        values["num_per_pallet"] = objects.num_per_layer * values["layers"]
        return values

    class Config:
        extra = Extra.allow
        orm_mode = True
