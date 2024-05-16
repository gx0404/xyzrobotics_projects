# !/usr/bin/env python3
# coding=utf-8

import requests


def test_topic_list():
    """
    Test palletize api topic_list.
    """
    url = "http://127.0.0.1:7002/api/dpt/topic_list"
    response = requests.get(url).json()
    assert response["code"] == 0
    resdata = [
        {
            "topic": "/real_planning_environment_marker_topic",
            "type": "visualization_msgs/MarkerArray",
        },
        {
            "topic": "/booked_items_marker_topic",
            "type": "visualization_msgs/MarkerArray",
        },
    ]
    assert response["data"] == resdata


def test_add_box():
    """
    Test palletize api add_box.
    """
    url = "http://127.0.0.1:7002/api/dpt/add_box"
    response = requests.get(url).json()
    assert response["code"] == 0


def test_remove_box():
    """
    Test palletize api remove_box.
    """
    url = "http://127.0.0.1:7002/api/dpt/remove_box"
    response = requests.get(url).json()
    assert response["code"] == 0


def test_clear_env():
    """
    Test palletize api clear_env.
    """
    url = "http://127.0.0.1:7002/api/dpt/clear_env"
    response = requests.get(url).json()
    assert response["code"] == 0
