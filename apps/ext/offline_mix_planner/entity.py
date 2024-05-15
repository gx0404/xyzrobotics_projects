#  -*- coding: utf-8 -*-
#  Copyright (c) XYZ Robotics Inc. - All Rights Reserved
#  Unauthorized copying of this file, via any medium is strictly prohibited
#  Proprietary and confidential
#  Author: Yuhang Wu <yuhang.wu@xyzrobotics.ai>, 2022/8/31 下午1:31
from datetime import datetime
from typing import List, Iterator, Optional

from pydantic import BaseModel, Field

from .enums import PlanningStatus


class Position(BaseModel):
    """位置信息."""

    x: float
    y: float
    z: float


class Orientation(BaseModel):
    """欧拉角表示的姿态信息."""

    x: float
    y: float
    z: float
    w: float


class Box(BaseModel):
    """Box的定义."""

    box_uid: int
    sku_id: Optional[str] = None
    length: Optional[float] = None
    width: Optional[float] = None
    height: Optional[float] = None
    weight: Optional[float] = None
    position: Position
    orientation: Orientation


class BoxResult(BaseModel):
    """箱子规划结果."""

    box: Box = Field(..., alias="box_info")

    class Config:
        allow_population_by_field_name = True


class PalletResult(BaseModel):
    """一个托盘上的规划结果."""

    box_results: List[BoxResult] = Field(..., alias="box_res", title="箱子规划结果")

    class Config:
        allow_population_by_field_name = True


class BatchResult(BaseModel):
    """批次的定义."""

    pallet_results: List[PalletResult] = Field(..., alias="pallet_res", title="托盘规划结果")

    class Config:
        allow_population_by_field_name = True


class PlanningResult(BaseModel):
    """规划结果."""

    batch_results: List[BatchResult] = Field(..., alias="batch_results", title="批次结果")
    pallet_num: int = Field(..., description="托盘数量")
    chosen_comb: list = Field(..., description="")
    sku_pallet: dict = Field(..., description="")

    def traverse_box(self) -> Iterator[BoxResult]:
        """遍历所有的BoxResult."""
        for batch in self.batch_results:
            for pallet in batch.pallet_results:
                for box in pallet.box_results:
                    yield box

    def traverse_pallet(self) -> Iterator[PalletResult]:
        """遍历所有的PalletResult."""
        for batch in self.batch_results:
            for pallet in batch.pallet_results:
                yield pallet

    def traverse_batch(self) -> Iterator[BatchResult]:
        """遍历所有的BatchResult."""
        for batch in self.batch_results:
            yield batch

    class Config:
        allow_population_by_field_name = True


class PlanningResultRecord(BaseModel):
    id: int
    order_id: str
    status: PlanningStatus
    result: Optional[PlanningResult] = None
    start_time: Optional[datetime] = None
    end_time: Optional[datetime] = None
    create_time: Optional[datetime] = None
    update_time: Optional[datetime] = None

    class Config:
        orm_mode = True


if __name__ == "__main__":
    t = [
        {
            "pallet_res": [
                {
                    "box_res": [
                        {
                            "box_info": {
                                "box_uid": 2,
                                "sku_id": "6907179110814",
                                "length": 0.6,
                                "width": 0.35,
                                "height": 0.3,
                                "weight": 4.5,
                                "position": {
                                    "x": -0.035,
                                    "y": -0.105,
                                    "z": 0.3},
                                "orientation": {
                                    "x": 0.0,
                                    "y": 0.0,
                                    "z": 0.0,
                                    "w": 1.0},
                            }
                        }
                    ]
                },
                {
                    "box_res": [
                        {
                            "box_info": {
                                "box_uid": 3,
                                "sku_id": "6907179110814",
                                "length": 0.6,
                                "width": 0.35,
                                "height": 0.3,
                                "weight": 4.5,
                                "position": {
                                    "x": -0.035,
                                    "y": -0.105,
                                    "z": 0.3},
                                "orientation": {
                                    "x": 0.0,
                                    "y": 0.0,
                                    "z": 0.0,
                                    "w": 1.0},
                            }
                        }
                    ]
                },
                {
                    "box_res": [
                        {
                            "box_info": {
                                "box_uid": 0,
                                "sku_id": "6904579670800",
                                "length": 0.4,
                                "width": 0.35,
                                "height": 0.3,
                                "weight": 5.0,
                                "position": {"x": -0.16, "y": -0.08, "z": 0.3},
                                "orientation": {
                                    "x": 0.0,
                                    "y": 0.0,
                                    "z": 0.7071068,
                                    "w": 0.7071068,
                                },
                            }
                        }
                    ]
                },
                {
                    "box_res": [
                        {
                            "box_info": {
                                "box_uid": 1,
                                "sku_id": "6904579670800",
                                "length": 0.4,
                                "width": 0.35,
                                "height": 0.3,
                                "weight": 5.0,
                                "position": {
                                    "x": -0.135,
                                    "y": -0.105,
                                    "z": 0.3},
                                "orientation": {
                                    "x": 0.0,
                                    "y": 0.0,
                                    "z": 0.0,
                                    "w": 1.0},
                            }
                        }
                    ]
                },
            ]
        }
    ]
    plan_result = PlanningResult.parse_obj(
        {"batch_results": t, "pallet_num": 1}
    )
    print(plan_result)
