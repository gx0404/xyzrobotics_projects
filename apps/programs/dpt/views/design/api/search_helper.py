#  Copyright (c) XYZ Robotics Inc. - All Rights Reserved.
#  Unauthorized copying of this file, via any medium is strictly prohibited.
#  Proprietary and confidential.
#  Author: Shuanglong Wang <shuanglong.wang@xyzrobotics.ai>, 3. 2021.
from enum import Enum
from collections import OrderedDict
from sqlalchemy import and_, text
from flask_restful import marshal

from ..api.page_helper import PageHelper


class QueryType(Enum):
    NORMAL = 0
    FUZZY = 1
    RANGE = 2


class SearchHelper:
    """
    use for search filter by property
    """
    search_parser = None
    Model = None

    def __init__(self, model, search_parser):
        self.search_parser = search_parser
        self.Model = model

    def filter_all(self, props, fields):
        """
        filter all results from Model
        :param props: property that need to filter
        :param fields: return fields
        """
        args = self.search_parser.parse_args(strict=True)
        query_set = self.Model.query
        query_set = self.filter(props, query_set)
        query_set = query_set.order_by(text("-id"))
        return self.query_set_wrapper(query_set, fields)

    def query_set_wrapper(self, query_set, fields):
        """
        wrap query set to dict
        """
        args = self.search_parser.parse_args(strict=True)
        if args['page'] and args['per_page']:
            pa = query_set.paginate(args['page'], args['per_page'])
            return PageHelper.page_marshal(pa, fields)
        else:
            all_set = query_set.all()
            item_list = marshal(all_set, fields)
            res = OrderedDict([
                ('records_total', len(all_set)),
                ('list', item_list),
            ])
            return res

    def filter(self, props, query_set):
        """
        filter results from query set
        :param props: property that need to filter
        :param query_set
        :return query_set
        """
        args = self.search_parser.parse_args(strict=True)
        print("---ARGS", args)
        rule_list = []
        for prop, q_type in props.items():
            if q_type is QueryType.NORMAL:
                if args[prop] is not None and args[prop] != '':
                    rule_list.append(getattr(self.Model, prop) == args[prop])
            elif q_type is QueryType.FUZZY:
                if args[prop] is not None and args[prop] != '':
                    rule_list.append(getattr(self.Model, prop).like(u"%{}%".format(args[prop])))
            elif q_type is QueryType.RANGE:
                if args[prop + "_min"]:
                    rule_list.append(getattr(self.Model, prop) >= args[prop + "_min"])
                if args[prop + "_max"]:
                    rule_list.append(getattr(self.Model, prop) <= args[prop + "_max"])
        rule = and_(*rule_list)
        query_set = query_set.filter(rule)
        order_column, desc = args['order_column'], args['desc']
        if order_column and hasattr(self.Model, order_column):
            query_set = query_set.order_by(text(("-" if desc else "") + order_column))
        return query_set
