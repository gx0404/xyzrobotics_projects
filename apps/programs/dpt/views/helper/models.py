# from __future__ import unicode_literals
from sqlalchemy import Column, Integer, Text, String, DateTime, ForeignKey
from sqlalchemy.dialects.mysql import LONGTEXT
from apps.models import db


def init_app(app):
    db.init_app(app)


def to_dict(self):
    return {c.name: getattr(self, c.name, None) for c in self.__table__.columns}


# db.Model.to_dict = to_dict
db.Model.__table_args__ = {
    "mysql_charset": "utf8"
}


class ActionLabel(db.Model):
    __tablename__ = 'action_label'
    id = Column(Integer, primary_key=True, autoincrement=True)
    mode = Column(String(45))
    tool_id = Column(Integer)
    topple_strength_degree = Column(Integer)
    assist_task_id = Column(Integer, ForeignKey('assist_task.task_id'))
    assist_task = db.relationship('AssistTask', backref=db.backref('action_labels'))


class AssistTask(db.Model):
    __tablename__ = 'assist_task'
    task_id = Column(Integer, primary_key=True, autoincrement=True)
    assistant_name = Column(Text)
    state = Column(Text)
    msg = Column(Text)
    timestamp_request = Column(DateTime)
    timestamp_response = Column(DateTime)
    timestamp_assign = Column(DateTime)
    timestamp_finish = Column(DateTime)
    label_method = Column(Text)
    assigned_labelers = Column(Text)
    error_msg = Column(Text)
    error_code = Column(Integer)


class ExpImage(db.Model):
    __tablename__ = 'exp_image'
    image_id = Column(Integer, primary_key=True, autoincrement=True)
    task_id = Column(Integer, ForeignKey('assist_task.task_id'))
    task = db.relationship('AssistTask', backref=db.backref('images'))
    image_url = Column(Text)
    camera_id = Column(Text)
    img_type = Column(String(255))
    img_height = Column(Integer)
    img_width = Column(Integer)
    annotation_json_string = Column(LONGTEXT)


class ToolInformation(db.Model):
    __tablename__ = 'tool_information'
    id = Column(Integer, primary_key=True, autoincrement=True)
    tool_id = Column(Integer, nullable=False)
    tool_name = Column(Text, nullable=False)
    img_path = Column(Text)
    is_used = Column(Integer)
