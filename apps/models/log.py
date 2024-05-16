# -*- coding: utf-8 -*-
from apps.models import db


def to_dict(self):
    res = {}
    for i in self.__dict__:
        if not i.startswith('_'):
            val = self.__dict__[i]
            if i.endswith('_'):
                i = i[:-1]
            # res[i] = val.encode("utf-8") if hasattr(val, "startswith") else val
            res[i] = val

    return res


db.Model.to_dict = to_dict
db.Model.__table_args__ = {
    "mysql_charset": "utf8"
}


class Log(db.Model):
    __tablename__ = 'log'
    id_ = db.Column(db.Integer, primary_key=True, autoincrement=True, name="id")
    msg_type = db.Column(db.String(10))
    code = db.Column(db.String(10))
    class_ = db.Column(db.String(30), name="class")
    tag = db.Column(db.String(30))
    object_ = db.Column(db.String(30), name="object")
    tool = db.Column(db.String(30))
    worksapce = db.Column(db.String(30))
    zh_msg = db.Column(db.Text)
    en_msg = db.Column(db.Text)
    ja_msg = db.Column(db.Text)
    zh_tip = db.Column(db.Text)
    en_tip = db.Column(db.Text)
    ja_tip = db.Column(db.Text)
    timeout = db.Column(db.Integer)
    timestamp = db.Column(db.Integer)
