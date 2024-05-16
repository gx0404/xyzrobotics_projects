import time
from apps.models import db, Log


def save_log(data):
    log = Log(
        msg_type=data['msg_type'],
        code=data['code'],
        class_=data.pop('class', "sys"),
        tag=data.pop('tag', "sys"),
        object_=data.pop('object', ""),
        tool=data.pop('tool', ""),
        worksapce=data.pop('worksapce', ""),
        zh_msg=data.get('zh_msg', ""),
        en_msg=data.get('en_msg', ""),
        ja_msg=data.get('ja_msg', ""),
        zh_tip=data.get('zh_tip', ""),
        en_tip=data.get('en_tip', ""),
        ja_tip=data.get('ja_tip', ""),
        timeout=data.get('timeout', None),
        timestamp=data.pop('timestamp', time.time())
    )
    db.session.add(log)
    db.session.commit()
