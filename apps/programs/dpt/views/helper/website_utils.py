#!/usr/bin/env python
"""
Copyright (c) XYZ Robotics Inc. - All Rights Reserved.
Unauthorized copying of this file, via any medium is strictly prohibited.
Proprietary and confidential.
Author: Michael Yung Shan Su <michael.su@xyzrobotics.ai>, Dec. 2019.
"""
import datetime
from apps.models import db
from apps.programs.dpt.views.helper.models import AssistTask, ExpImage
from apps.utils.upload_manager import check_upload_image, save_upload_file, get_file_real_path


def save_to_database(request_help_data, files):
    """Save  request help data to database

    """
    # Check file is jpg file
    if not check_upload_image(files["img"]):
        return None

    print(request_help_data)
    # Update assist_task table
    assist_task = AssistTask(state="processing",
                             timestamp_request=datetime.datetime.now(),
                             msg=request_help_data.get("msg", None),
                             label_method=request_help_data["label_method"])

    db.session.add(assist_task)

    # Update exp_image table and download images
    exp_img = None
    # Save image_file
    image_url = save_upload_file(
        files["img"],
        filename="helper_" + request_help_data["image_path"].replace("/", "_")
    )
    print("GET IMG URL:", image_url)

    if request_help_data["label_method"] == "box_label":
        exp_img = ExpImage(image_url=image_url,
                           camera_id=request_help_data["camera_id"],
                           annotation_json_string=request_help_data.get("annotation_json_string", ""),
                           task=assist_task)
    db.session.add(exp_img)

    assist_task.state = "unassigned"
    db.session.commit()
    db.session.flush()

    return assist_task.task_id


def path_convert(path):
    """ Convert file path to final save path

    """
    path = get_file_real_path(path)
    return path
