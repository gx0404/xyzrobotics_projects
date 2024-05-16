# -*- coding: utf-8 -*-
"""
Copyright (c) XYZ Robotics Inc. - All Rights Reserved
Unauthorized copying of this file, via any medium is strictly prohibited
Proprietary and confidential
Author: Yuhang Wu <yuhang.wu@xyzrobotics.ai>, 2022-07-25
"""
import uuid
from io import BytesIO
from typing import Any, List, Literal, Optional, Union

import numpy as np
import pandas as pd
import pydantic.error_wrappers
import tf.transformations as tfm
import xlsxwriter
from pandas import DataFrame
from werkzeug.datastructures import FileStorage
from xyz_homogeneous_bin_packing import (
    HomogeneousBinPacking as HBP,
    NaiveStabilityChecker,
    OrientedBinPacking as OBP,
)

from apps.exceptions import XYZBaseError, XYZValidationError
from apps.log import hmi_log
from apps.programs.dpt.views.design_v2.entity import (
    BoxEntity,
    PalletEntity,
    PlanEntity,
    Layout,
)
from apps.programs.dpt.views.design_v2.enums import BarcodeDirection, Flip, Mirror
from apps.programs.dpt.views.design_v2.schemas import BoxCreateSchema


def layout_shrink(layouts):
    #  x,y,z,qx,qy,qz,qw,id ->  x,y,rotation , m -> mm
    return [
        [[box[0], box[1], q_to_rotation(box[:7])] for box in layout]
        for layout in layouts
    ]


def layout_expand(layouts):
    """transform layout json data , mm -> m"""
    # x,y,rotation -> x,y,z,qx,qy,qz,qw,id
    return [
        np.array([[box[0], box[1], 0] + list(rotation_to_q(box[2])) for box in layout])
        for layout in layouts
    ]


def rotation_to_q(rotation):
    """
    将HMI反馈的1，2，3，4，转换为四元数，然后给XTF调用使用
    https://xyz-robotics.atlassian.net/wiki/spaces/depalletizeproduct/pages/475562620/HMI+xlhb
    Args:
        rotation: int, 1,2,3,4数字
                1代表0度（纸箱不旋转）
                2代表90度（纸箱逆时针旋转90度）
                3代表180度（纸箱逆时针旋转180度）
                4代表-90度（纸箱顺时针旋转90度）
    Returns:
         四元数，qx,qy,qz,qw,表示旋转，list
    """
    if rotation == 1:
        # 0
        return 0.0, 0.0, 0.0, 1.0  # x,y,z,w
    elif rotation == 2:
        # 90
        return 0.0, 0.0, 0.7071067811865475, 0.7071067811865476  # x,y,z,w
    elif rotation == 3:
        # 180
        return 0, 0, 1.0, 0.0  # x,y,z,w
    elif rotation == 4:
        # -90
        return 0.0, 0.0, -0.7071067811865475, 0.7071067811865476  # x,y,z,w
    else:
        raise TypeError("垛型规划中参数输入错误")


def q_to_rotation(q):
    """
    将pose中的四元数旋转转换为1，2，3，4，然后反馈给HMI端使用
    https://xyz-robotics.atlassian.net/wiki/spaces/depalletizeproduct/pages/475562620/HMI+xlhb
    Args:
        q: list,   x,y,z,qx,qy,qz,qw

    Returns:
        int, 1,2,3,4数字
                1代表0度（纸箱不旋转）
                2代表90度（纸箱逆时针旋转90度）
                3代表180度（纸箱逆时针旋转180度）
                4代表-90度（纸箱顺时针旋转90度）
    """
    # FIXME(YuhangWu): xyz_motion库终止异常.
    #   当程序终止时，该库会抛出-6退出码，并抛出异常： double free or corruption (fasttop)
    #   详情见: https://xyz-robotics.atlassian.net/browse/MQWR-1115?atlOrigin=eyJpIjoiN2Q4ZGNlNWMyMTE0NDcxOThjZTQyMzBmYjc5ODdiNjgiLCJwIjoiaiJ9
    try:
        from xyz_motion import SE3

    # 这里除了ImportError异常，还有其他异常，所以此处捕获所有异常.
    except Exception as err:
        raise Exception("xyz_motion加载失败.") from err

    vp = SE3(q)
    vquat = tfm.quaternion_from_matrix(vp.homogeneous)
    box_r = tfm.euler_from_quaternion(vquat)
    box_rotation = np.rad2deg(box_r)
    box_angle = np.float(box_rotation[2])
    if -2.0 < box_angle < 2.0:
        return 1
    elif 88.0 < box_angle < 92.0:
        return 2
    elif 178.0 < box_angle < 182.0:
        return 3
    elif -92.0 < box_angle < -88.0:
        return 4
    else:
        raise TypeError("垛型规划中参数输入错误")


def generate_suggest_layout(
    box: BoxEntity,
    pallet: PalletEntity,
    barcode_direction: BarcodeDirection,
    mirror: Mirror,
    guillotine_packing: Literal[0, 1],
    flip: Optional[Flip] = None,
):
    """生成建议垛型
        box (BoxEntity): 箱子实体
        pallet (PalletEntity): 托盘实体
        barcode_direction (BarcodeDirection): 条码方向
        mirror (Mirror): 镜像
        guillotine_packing (Literal[0, 1]): 是否回型

    Returns:
        list: 建议垛型
    """
    solver = get_solver(barcode_direction, guillotine_packing, box, pallet)
    n = solver.solve()
    if solver.n_layers == 0 or n <= 0:
        hmi_log.error(f"box: {box}, pallet: {pallet}")
        raise XYZBaseError(
            error_message="No box pose can be returned. Please check the pallet dimensions and the box dimensions"
        )

    odd_poses, even_poses = solver.generate_both_layouts(mirror)

    if flip != Flip.DEFAULT:
        even_poses, odd_poses = get_flip(
            barcode_direction, even_poses, odd_poses, flip, solver
        )

    layouts = layout_shrink([odd_poses, even_poses])
    # data = json.dumps(layouts, cls=NumpyEncoder)
    return layouts


def get_flip(
    barcode_direction: BarcodeDirection, even_poses, odd_poses, flip: Flip, solver
):
    odd_poses = np.array(odd_poses)
    even_poses = np.array(even_poses)
    if flip == Flip.X:
        if barcode_direction == BarcodeDirection.DEFAULT:
            odd_poses = odd_poses * [-1, 1, 1, 1, 1, 1, 1]
            even_poses = even_poses * [-1, 1, 1, 1, 1, 1, 1]
        else:
            odd_poses, even_poses = solver.flip(flip)
    elif flip == Flip.Y:
        if barcode_direction == BarcodeDirection.DEFAULT:
            odd_poses = odd_poses * [1, -1, 1, 1, 1, 1, 1]
            even_poses = even_poses * [1, -1, 1, 1, 1, 1, 1]
        else:
            odd_poses, even_poses = solver.flip(flip)
    else:
        raise XYZBaseError(error_message="error flip parameter")
    return even_poses, odd_poses


def get_solver(
    barcode_direction: BarcodeDirection,
    guillotine_packing: Literal[0, 1],
    box: BoxEntity,
    pallet: PalletEntity,
):
    """获取垛型规划器
        barcode_direction (BarcodeDirection): 条码方向
        guillotine_packing (Literal[0, 1]): 是否回型
        box (BoxEntity): 箱子实体
        pallet (PalletEntity): 托盘实体

    Returns:
        solver: 垛型规划器
    """
    if barcode_direction in [
        BarcodeDirection.ONE_LONG_SIDE,
        BarcodeDirection.TWO_LONG_SIDE,
    ]:
        # 条码在长边
        return OBP(
            [box.length, box.width, box.height],
            [pallet.length, pallet.width, pallet.max_height],
            spacing=False,
            guillotine_packing=bool(guillotine_packing),
            long_edge_outside=True,
        )

    elif barcode_direction in [
        BarcodeDirection.ONE_SHORT_SIDE,
        BarcodeDirection.TWO_SHORT_SIDE,
    ]:
        # 条码在短边
        return OBP(
            [box.length, box.width, box.height],
            [pallet.length, pallet.width, pallet.max_height],
            spacing=False,
            guillotine_packing=bool(guillotine_packing),
            long_edge_outside=False,
        )

    else:
        # 无条码朝向要求，自由垛型
        return HBP(
            [box.length, box.width, box.height],
            [pallet.length, pallet.width, pallet.max_height],
            spacing=False,
        )


def validate_plan(
    box: BoxEntity,
    pallet: PalletEntity,
    layers: int,
    layout: Optional[Layout] = None,
) -> None:
    """校验规划.

    Args:
        box(BoxEntity): 纸箱对象.
        pallet(PalletEntity): 托盘对象.
        layers(int): 层数.
        layout(Layout): 规划方案.

    Raises:
        XYZBaseError: 自定义异常.
        XYZValidationError: 数据校验异常.
    """
    if layout:
        layout = layout.__root__
        layout = layout_expand(layout)
        if len(layout) == 1:
            layer_0 = layer_1 = layout[0]
        elif len(layout) == 2:
            layer_0, layer_1 = layout[0], layout[1]
        else:
            raise XYZBaseError(error_message="layout format error")

        if not layer_1.any():
            layer_1 = layer_0

        try:
            sc = NaiveStabilityChecker(
                layer_0,
                layer_1,
                [box.length, box.weight, box.height],
                [pallet.length, pallet.width, pallet.max_height],
            )
            passed = sc.stability_check()
        except IndexError:
            raise XYZValidationError(error_message="垛型规划稳定性检查不通过") from None

    if layers and layers * box.height > pallet.max_height:
        raise XYZValidationError(error_message="自定义层数 * 箱子高度超过最大可用高度")


def parse_box_file(file: Union[FileStorage, bytes]) -> List[BoxCreateSchema]:
    """解析纸箱文件.

    使用pandas解析纸箱文件.

    Examples:
        >>> with open("/home/xyz/Desktop/纸箱导入-测试.xlsx", "rb") as f:
        ...     content = f.read()
        ...     parse_box_file(content)

    Args:
        file(FileStorage): 文件对象.

    Returns:
        List[BoxCreateSchema]: 一批创建纸箱模式对象.
    """
    df: DataFrame
    if isinstance(file, FileStorage):
        file = file.read()
    df = pd.read_excel(io=BytesIO(initial_bytes=file), engine="openpyxl")
    # replace "nan" to None
    df = df.replace({np.nan: None})
    dataset = df.to_dict("records")
    hmi_log.debug(f"解析Box文件: {dataset}")
    ret = []
    for data in dataset:
        values = list(data.values())
        try:
            u = uuid.uuid4().__str__()
            ret.append(
                BoxCreateSchema(
                    length=values[0],
                    width=values[1],
                    height=values[2],
                    weight=values[3],
                    name=values[4] or u,
                    scan_code=values[5] or u,
                )
            )
        except pydantic.error_wrappers.ValidationError as err:
            raise XYZValidationError(error_message="部分数据缺失, 请检测数据完整性.") from err
        except Exception as err:
            raise XYZValidationError(error_message="纸箱格式错误，请按照纸箱模板规范填写.") from err
    return ret


def check_duplicate_box(boxes: List[BoxCreateSchema]) -> Optional[BoxCreateSchema]:
    """校验是否存在重复箱子.

    Args:
        boxes: 一批待创建的箱子.

    Returns:
        BoxCreateSchema or None: 没有重复箱子返回None，有重复箱子即返回该重复箱子对象.
    """
    # 判断是否存在重复的长宽高.
    s = set()
    for box in boxes:
        unq_key = f"{box.length}{box.width}{box.height}"
        old_length = len(s)
        s.add(unq_key)
        if len(s) == old_length:
            # 长度相同，说明没有新增，即存在重复数据.
            return box


def generate_plan_workbook(
    entities: List[PlanEntity], filename: Any, options: Optional[dict] = None
) -> xlsxwriter.Workbook:
    """构建workbook对象."""
    workbook = xlsxwriter.Workbook(filename, options=options)
    # Add a format for the headings.
    worksheet = workbook.add_worksheet("Sheet1")
    bold = workbook.add_format({"bold": 1})
    headers = [
        "规划编号",
        "层数",
        "每层个数",
        "整托个数",
        "覆盖率",
        "垛型图",
        "纸箱编号",
        "纸箱长(mm)",
        "纸箱宽(mm)",
        "纸箱高(mm)",
        "纸箱重量(kg)",
        "托盘编号",
        "托盘名称",
        "托盘长(mm)",
        "托盘宽(mm)",
        "托盘高(mm)",
        "托盘最大高度(mm)",
        "托盘最大载重(kg)",
    ]
    worksheet.set_column(first_col=5, last_col=5, width=20)
    worksheet.set_default_row(height=45)
    worksheet.set_row(row=0, height=20)
    worksheet.write_row("A1", headers, bold)
    for i, entity in enumerate(entities):
        box = entity.box
        pallet = entity.pallet
        coverage = "{:.2f}%".format(
            (box.length * box.width * entity.num_per_layer)
            / (pallet.length * pallet.width)
            * 100
        )
        row = [
            entity.id,
            entity.layers,
            entity.num_per_layer,
            entity.num_per_pallet,
            coverage,
            "",
            box.id,
            box.length,
            box.width,
            box.height,
            box.weight,
            pallet.id,
            pallet.name,
            pallet.length,
            pallet.width,
            pallet.height,
            pallet.max_height,
            pallet.max_weight,
        ]
        worksheet.write_row(row=i + 1, col=0, data=row)
        if entity.image_url:
            filename = str(entity.image_real_path)
            worksheet.insert_image(
                row=i + 1,
                col=5,
                filename=filename,
                options={
                    "x_scale": 0.2,
                    "y_scale": 0.15,
                    "x_offset": 0,
                    "y_offset": 0,
                    "positioning": 1,
                },
            )
    workbook.close()
    return workbook


def generate_plan_excel_to_bytes(entities: List[PlanEntity]) -> BytesIO:
    """生成垛型规划表格文件，字节流的形式."""
    output = BytesIO()
    generate_plan_workbook(entities, filename=output, options={"in_memory": True})
    output.seek(0)
    return output


def generate_plan_excel_to_file(entities: List[PlanEntity]) -> None:
    """生成垛型规划表格文件.

    Examples:
        >>> from apps.programs.dpt.views.design_v2.entity import PlanEntity, BoxEntity, PalletEntity
        >>> box = BoxEntity(id=1, name="1", length=1000, width=1000, height=1000, weight=20)
        >>> pallet = PalletEntity(id=1, name="1", length=2000, width=1500, height=2000, max_height=3000, max_weight=10000, pallet_type=False, thickness=None)
        >>> plan = PlanEntity(
        ...     id=1,
        ...     box=box,
        ...     pallet=pallet,
        ...     guillotine_packing=0,
        ...     barcode_direction=0,
        ...     mirror=0,
        ...     flip="x",
        ...     layout="[[[499.99997999999994,1.5000000075815478e-05,1],[-500.00002000000006,1.5000000075815478e-05,1]],[[499.99997999999994,1.5000000075815478e-05,1],[-500.00002000000006,1.5000000075815478e-05,1]]]",
        ...     layers=2,
        ...     image_url="1.png"
        ... )
        >>> generate_plan_excel_to_file([plan])

    """
    generate_plan_workbook(entities, filename="demo.xlsx", options={"in_memory": True})
