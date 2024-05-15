from collections import OrderedDict

from flask_restful import marshal, inputs


class PageHelper:
    def __init__(self):
        pass

    @staticmethod
    def page_parser(parser):
        search_parser = parser.copy()
        search_parser.add_argument('per_page', type=int)
        search_parser.add_argument('page', type=int)
        search_parser.add_argument('order_column', type=str)
        search_parser.add_argument('desc', type=inputs.boolean)
        return search_parser

    @staticmethod
    def page_marshal(paginate, m_fields):
        items = paginate.items
        item_list = marshal(items, m_fields)

        res = OrderedDict([
            ('page', paginate.page),
            ('per_page', paginate.per_page),
            ('pages_total', paginate.pages),
            ('records_total', paginate.total),
            ('list', item_list),
        ])
        return res
