# -*- coding: utf-8 -*-
"""
Copyright (c) XYZ Robotics Inc. - All Rights Reserved
Unauthorized copying of this file, via any medium is strictly prohibited
Proprietary and confidential
Author: Yuhang Wu <yuhang.wu@xyzrobotics.ai>, 2022-07-26
"""
from typing import Iterable, List, Optional, Union

import sqlalchemy.exc
from sqlalchemy import or_
from sqlalchemy.orm import Query
from sqlalchemy.orm.scoping import ScopedSession

from apps.base_app.crud import CRUDBase
from apps.exceptions import XYZIntegrityError, XYZNotFoundError
from apps.ext.datastructs.sku import SKUProto
from apps.log import hmi_log
from apps.models import db
from apps.programs.dpt.views.design.models import Box, Design as Plan, Pallet
from apps.search_filter import QueryFilter
from apps.utils.upload_manager import save_upload_file
from .entity import BoxEntity, PalletEntity, PlanEntity
from .schemas import (
    BoxBatchCreateSchema,
    BoxCreateSchema,
    BoxQuerySchema,
    BoxUpdateSchema,
    PalletCreateSchema,
    PalletQuerySchema,
    PalletUpdateSchema,
    PlanCreateSchema,
    PlanPatchSchema,
    PlanQuerySchema,
    PlanUpdateSchema,
)


class CRUDPlan(
    CRUDBase[Plan, PlanEntity, PlanQuerySchema, PlanCreateSchema, PlanUpdateSchema]
):
    def _get_raw_model(
        self, session: ScopedSession = db.session, *, pk: int
    ) -> Optional[Plan]:
        """获取原始记录.

        Args:
            session(ScopedSession): 数据库会话对象.
            pk(int): 主键值.

        Returns:
            DBModelType or None: 返回数据库模型对象, 不存在则返回None.
        """
        return (
            session.query(self.model)
            .filter(self.model.id == pk, self.model.is_del == False)
            .first()
        )

    def search(
        self, session: ScopedSession = db.session, *, query: PlanQuerySchema
    ) -> List[PlanEntity]:
        sa_filters = query.to_sa_filters()
        collation = query.to_sa_collation()
        queryset = (
            session.query(Plan)
            .filter(*sa_filters)
            .order_by(*collation)
            .limit(query.page_size)
            .offset((query.page - 1) * query.page_size)
        )
        hmi_log.debug(f"{queryset}")
        return [self.entity.from_orm(q) for q in queryset]

    def search_by_box(
        self, session: ScopedSession = db.session, *, query: PlanQuerySchema
    ) -> List[PlanEntity]:
        """通过纸箱搜索规划记录."""
        sub_filters = query.to_sa_sub_filters()
        subquery = session.query(Box.id).filter(*sub_filters).subquery()
        collation = query.to_sa_collation()
        queryset = (
            session.query(self.model)
            .filter(self.model.box_id.in_(subquery), self.model.is_del == False)
            .order_by(*collation)
            .limit(query.page_size)
            .offset((query.page - 1) * query.page_size)
        )
        hmi_log.debug(f"{queryset}")
        return [self.entity.from_orm(q) for q in queryset]

    def get_by_sku_id(self, sku_id: str) -> Optional[PlanEntity]:
        """根据纸箱编号获取规划记录"""
        session: ScopedSession = db.session
        # sku_id 对应数据库中的 box.name
        query = PlanQuerySchema(
            filters=[
                QueryFilter(field="name", op="eq", value=sku_id),
            ]
        )
        sub_filters = query.to_sa_sub_filters()
        subquery = session.query(Box.id).filter(*sub_filters).subquery()
        queryset = (
            session.query(self.model)
            .filter(self.model.box_id.in_(subquery), self.model.is_del == False)
            .offset((query.page - 1) * query.page_size)
        )
        return self.entity.from_orm(queryset.first())

    def get_by_sku(self,  sku: SKUProto) -> Optional[PlanEntity]:
        """根据纸箱获取规划记录."""
        session: ScopedSession = db.session
        query = PlanQuerySchema(
            filters=[
                QueryFilter(field="length", op="eq", value=sku.length),
                QueryFilter(field="width", op="eq", value=sku.width),
                QueryFilter(field="height", op="eq", value=sku.height),
            ]
        )
        sub_filters = query.to_sa_sub_filters()
        subquery = session.query(Box.id).filter(*sub_filters).subquery()
        queryset = (
            session.query(self.model)
            .filter(self.model.box_id.in_(subquery), self.model.is_del == False)
            .offset((query.page - 1) * query.page_size)
        )
        return self.entity.from_orm(queryset.first())

    def count(
        self,
        session: ScopedSession = db.session,
        *,
        query: PlanQuerySchema,
    ) -> int:
        sub_filters = query.to_sa_sub_filters()
        subquery = session.query(Box.id).filter(*sub_filters).subquery()
        return (
            session.query(self.model)
            .filter(self.model.box_id.in_(subquery), self.model.is_del == False)
            .count()
        )

    def pagination(
        self,
        session: ScopedSession = db.session,
        *,
        query: PlanQuerySchema,
    ) -> dict:
        """分页查询"""
        return {
            "page": query.page,
            "page_size": query.page_size,
            "count": self.count(query=query),
            "data": self.search_by_box(query=query),
        }

    def batch_create(
        self, session: ScopedSession, create_list: List[PlanCreateSchema]
    ) -> List[PlanEntity]:
        """创建多个."""
        return [self.create(session, create) for create in create_list]

    def get_last(
        self, session: ScopedSession = db.session, *, pk: int
    ) -> Optional[int]:
        """获取上一个记录(按 id 降序排序).

        Args:
            session(ScopedSession): 数据库会话.
            pk(int): 主键值.

        Returns:
            PlanEntity: 一个规划记录实体对象.
        """
        if (
            result := session.query(self.model.id)
            .filter(self.model.id > pk, self.model.is_del == False)
            .first()
        ):
            return result.id

    def get_next(
        self, session: ScopedSession = db.session, *, pk: int
    ) -> Optional[int]:
        """获取下一个记录(按 id 降序排序).

        Args:
            session(ScopedSession): 数据库会话.
            pk(int): 主键值.

        Returns:
            PlanEntity: 一个规划记录实体对象.
        """
        if (
            result := session.query(self.model.id)
            .order_by(self.model.id.desc())
            .filter(self.model.id < pk, self.model.is_del == False)
            .first()
        ):
            return result.id

    def patch_update(
        self, session: ScopedSession, patch_update: PlanPatchSchema, pk: int
    ) -> PlanEntity:
        """

        Args:
            session(ScopedSession): 数据库会话对象.
            patch_update(PlanPatchSchema): patch模式对象.
            pk(int): 主键值.

        Returns:
            PlanEntity: 一个规划记录.

        Raises:
            XYZNotFoundError: 记录不存在.
        """
        instance = self._get_raw_model(session, pk=pk)
        if not instance:
            raise XYZNotFoundError(
                error_message=f"Retrieve failed, Not Found {self.entity.__name__}({pk})."
            )
        for key, val in patch_update.dict(exclude_none=True).items():
            setattr(instance, key, val)
        return self.entity.from_orm(instance)

    def is_exists_by_box(self, session: ScopedSession, box: BoxEntity) -> bool:
        """根据纸箱ID判断规划记录是否存在."""
        q = session.query(self.model.id).filter(
            self.model.box_id == box.id, self.model.is_del == False
        )
        return q.count() > 0

    def is_exists_by_pallet(self, session: ScopedSession, pallet: PalletEntity) -> bool:
        """根据托盘ID判断规划记录是否存在."""
        q = session.query(self.model.id).filter(
            self.model.pallet_id == pallet.id, self.model.is_del == False
        )
        return q.count() > 0

    def is_exists(
        self,
        session: ScopedSession = db.session,
        *,
        box_id: Optional[int] = None,
        pallet_id: Optional[int] = None
    ) -> bool:
        """根据纸箱ID和托盘ID判断规划记录是否存在."""
        q = session.query(self.model.id).filter(
            or_(
                self.model.box_id == box_id,
                self.model.pallet_id == pallet_id,
            ),
            self.model.is_del == False,
        )
        return q.count() > 0

    def get_no_image_plans(
        self, session: ScopedSession = db.session
    ) -> List[PlanEntity]:
        """获取没有数据的详情记录.

        .. versionchange:: 1.4.0
            仅查询未删除的记录
        """
        q = session.query(Plan).filter(
            self.model.image_url == None, self.model.is_del == False
        )
        return [self.entity.from_orm(q) for q in q.all()]

    def delete(self, session: ScopedSession, *, pk: int) -> Optional[PlanEntity]:
        """根据主键值删除记录，不存在则返回None.

        Args:
            session(ScopedSession): 会话对象.
            pk(int): 主键ID

        Returns:
            Entity: 一个实体对象.

        Raises:
            XYZNotFoundError: 记录不存在异常.
        """
        if model := self._get_raw_model(session=session, pk=pk):
            model.is_del = None
            return self.entity.from_orm(model)

    def delete_or_404(self, session: ScopedSession, *, pk: int) -> PlanEntity:
        """根据主键值删除记录，不存在则抛出404异常.

        Args:
            session(ScopedSession): 会话对象.
            pk(int): 主键ID

        Returns:
            Entity: 一个实体对象.

        Raises:
            XYZNotFoundError: 记录不存在异常.
        """
        if result := self._get_raw_model(session, pk=pk):
            result.is_del = None
            return self.entity.from_orm(result)
        else:
            raise XYZNotFoundError(
                error_message=f"Delete failed, Not Found {self.entity.__name__}({pk})."
            )


class CRUDPallet(
    CRUDBase[
        Pallet, PalletEntity, PalletQuerySchema, PalletCreateSchema, PalletUpdateSchema
    ]
):
    def delete(self, session: ScopedSession, *, pk: int) -> Optional[PalletEntity]:
        """根据主键值删除记录，不存在则返回None.

        Args:
            session(ScopedSession): 会话对象.
            pk(int): 主键ID

        Returns:
            Entity: 一个实体对象.

        Raises:
            XYZNotFoundError: 记录不存在异常.
        """
        if model := self._get_raw_model(session=session, pk=pk):
            model.is_del = None
            return self.entity.from_orm(model)


class CRUDBox(
    CRUDBase[Box, BoxEntity, BoxQuerySchema, BoxCreateSchema, BoxUpdateSchema]
):
    def create(self, session: ScopedSession, create: BoxCreateSchema) -> BoxEntity:
        data = create.dict()
        img_file = data.pop("img_file")
        box = self.model(**data)

        try:
            session.add(box)
            session.flush()
            if img_file:
                box.img_url = save_upload_file(
                    img_file, filename=f"dpt_box_{box.id}_{img_file.filename}"
                )

        except sqlalchemy.exc.IntegrityError as err:
            session.rollback()
            raise XYZIntegrityError(
                error_message=f"{self.entity.__name__} already exists: {str(err.args)}"
            ) from err
        return self.entity.from_orm(box)

    def batch_create(
        self,
        session: ScopedSession,
        create_list: BoxBatchCreateSchema,
    ) -> List[BoxEntity]:
        """批量创建纸箱.

        Args:
            session(ScopedSession): 数据库会话对象.
            create_list: 批创建模式类对象.

        Returns:
            List[BoxEntity]: 一批纸箱实体对象.

        Raises:
            XYZIntegrityError: 新增数据中包含重复数据.
        """
        dataset = create_list.dict()
        boxes = []
        for data in dataset["__root__"]:
            data.pop("img_file")
            box = self.model(**data)

            try:
                session.add(box)
                session.flush()
            except sqlalchemy.exc.IntegrityError as err:
                session.rollback()
                raise XYZIntegrityError(
                    error_message=f"{self.entity.__name__} already exists: {str(err.args)}"
                ) from err

            boxes.append(self.entity.from_orm(box))

        return boxes

    def delete_all_unused(
        self,
        session: ScopedSession,
        *,
        hard: bool = False,
    ) -> int:
        """删除所有未被使用的纸箱.

        select box.id, design.id
        from box left join design on box.id = design.box_id and box.is_del = false
        where design.box_id is null or design.is_del is null;

        Args:
            session: 会话对象

        Keyword Args:
            hard: 是否物理删除

        Returns:
            int: 成功删除的数量.
        """
        queryset: Query = session.query(self.model.id)
        subquery = queryset.outerjoin(Plan, self.model.id == Plan.box_id).filter(
            self.model.is_del == False,
            or_(
                Plan.box_id == None,
                Plan.is_del == None
            )
        )
        queryset = session.query(self.model).filter(self.model.id.in_(subquery.subquery()))
        return self._delete(queryset, hard)

    def remove_all_unused(
        self,
        session: ScopedSession,
        *,
        hard: bool = False,
    ) -> List[BoxEntity]:
        """移除所有未被使用的纸箱.

        功能和 `delete_all_unused` 一样，只是返回值不同.

        Args:
            session: 会话对象

        Keyword Args:
            hard: 是否物理删除

        Returns:
            List[BoxEntity]: 一组纸箱实体.
        """
        queryset: Query = session.query(self.model.id)
        subquery = queryset.outerjoin(Plan, self.model.id == Plan.box_id).filter(
            self.model.is_del == False,
            or_(
                Plan.box_id == None,
                Plan.is_del == None
            )
        )
        queryset = session.query(self.model).filter(self.model.id.in_(subquery.subquery()))
        results = [self.entity.from_orm(q) for q in queryset]
        self._delete(queryset, hard)
        return results

    def delete(self, session: ScopedSession, *, pk: int) -> Optional[BoxEntity]:
        """根据主键值删除记录，不存在则返回None.

        Args:
            session(ScopedSession): 会话对象.
            pk(int): 主键ID

        Returns:
            Entity: 一个实体对象.

        Raises:
            XYZNotFoundError: 记录不存在异常.
        """
        if model := self._get_raw_model(session=session, pk=pk):
            model.is_del = None
            return self.entity.from_orm(model)

    def remove_unused_by_id(
        self,
        session: ScopedSession,
        *,
        ids: Union[Iterable[int], int],
        hard: bool = False,
    ) -> Union[List[BoxEntity], BoxEntity]:
        """通过一个或多个主键id移除未使用的纸箱.

        select temp_box.id
        from (
            select box.id as id
            from box
            where box.id in (1,2,3,4) and box.is_del = false
        ) temp_box left join design on temp_box.id = design.box_id
        where design.box_id is null or design.is_del is null;

        Args:
            session(ScopedSession): 会话对象

        Keyword Args:
            ids(Iterable[int], int): 接收一个或多个id
            hard(bool): 是否为物理删除

        Returns:
            Union[List[BoxEntity], BoxEntity]: 如果传入一个 `id`, 则返回一个实体对象, 如果传入多个 `id`, 则返回一个组实体.
        """
        # TODO(YuhangWu): 应该将查询和删除整合到一个 SQL 语句中.
        # single_flag = False
        # if isinstance(ids, int):
        #     single_flag = True
        #     ids = [ids]

        # textual_sql = text(
        #     f"""
        #     select
        #         temp_box.id as id,
        #         temp_box.name as name,
        #         temp_box.legnth as length,
        #         temp_box.width as width,
        #         temp_box.height as height,
        #         temp_box.weight as weight
        #     from (
        #         select
        #             box.id as id,
        #             box.name as name,
        #             box.length as legnth,
        #             box.width as width,
        #             box.height as height,
        #             box.weight as weight
        #         from box
        #         where box.id in {str(tuple(ids))} and box.is_del = false
        #     ) temp_box left join design on temp_box.id = design.box_id
        #     where design.box_id is null or design.is_del is null;
        #     """
        # )
        # queryset: Query = session.query(self.model.id)
        # queryset.filter(self.model.id.in_(delete_box_ids))
        # entities = [self.entity.from_orm(q) for q in results]
        # row_count = self._delete(queryset, hard)
        # if row_count == 0:
        #     return []
        # return entities[0] if single_flag else entities


crud_plan = CRUDPlan(model=Plan, entity=PlanEntity)
crud_box = CRUDBox(model=Box, entity=BoxEntity)
crud_pallet = CRUDPallet(model=Pallet, entity=PalletEntity)
