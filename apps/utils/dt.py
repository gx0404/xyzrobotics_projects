import time
import datetime


def now_timestamp():
    return int(time.mktime(datetime.datetime.now().timetuple()))


def to_timestamp_from_str(date_str: str):
    """将符合日期的字符串转为时间戳.

    Args:
        date_str(str): 日期字符串.

    Returns:
        int: timestamp.
    """
    time_array = time.strptime(date_str, "%Y_%m_%d")
    timestamp = time.mktime(time_array)
    return int(timestamp)


def get_today_str(fmt="%Y%m%d"):
    return datetime.date.today().strftime(fmt)
