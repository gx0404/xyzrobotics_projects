import os
from typing import Optional, Union

from flask import current_app, send_from_directory, url_for
from werkzeug.datastructures import FileStorage
from werkzeug.utils import secure_filename

from apps.exceptions import XYZBaseError
from apps.log import hmi_log
from apps.settings import settings


def test_img(h):
    """ test file type by file head """

    def test_jpeg(h):
        """JPEG data in JFIF format"""
        if h[6:10] == 'JFIF'.encode():
            return 'jpeg'

    def test_exif(h):
        """JPEG data in Exif format"""
        if h[6:10] == 'Exif'.encode():
            return 'jpeg'

    def test_png(h):
        if h[:8] == b'\x89PNG\r\n\x1a\n':
            return 'png'

    img_type = test_jpeg(h) or test_exif(h) or test_png(h)
    if img_type:
        return img_type
    else:
        return None


def check_upload_image(content):
    """ Image format check, check upload flask file is image"""
    data = content.stream.read(32)
    content.stream.seek(0)
    return test_img(data)


def save_upload_file(content, filename=None):
    filename = secure_filename(filename or content.filename)
    hmi_log.info(f"secure file name: {filename}")
    if not os.path.exists(current_app.config['UPLOAD_FOLDER']):
        try:
            os.makedirs(current_app.config['UPLOAD_FOLDER'])
        except IOError as e:
            raise IOError(
                f"Create upload folder: {current_app.config['UPLOAD_FOLDER']} failed."
            ) from e

    content.save(os.path.join(current_app.config['UPLOAD_FOLDER'], filename))
    url = url_for('download_file', name=filename)
    hmi_log.info(f"saved file url: {url}")
    return url


def get_file_real_path(url):
    filename = url.replace(url_for('download_file', name=''), '')
    real_path = os.path.join(current_app.config['UPLOAD_FOLDER'], filename)
    return real_path


def download_file(name):
    return send_from_directory(current_app.config['UPLOAD_FOLDER'], name)


def save_file(
    save_dir: Union[str, os.PathLike],
    file: FileStorage,
    filename: Optional[str] = None
) -> str:
    filename = secure_filename(filename or file.filename)
    if not os.path.exists(save_dir):
        os.makedirs(save_dir)
    file.save(os.path.join(save_dir, filename))
    return url_for("static", filename=filename)


def save_image(
    file: FileStorage,
    filename: Optional[str] = None,
    dir_name: Optional[str] = None
) -> str:
    """存放图片

    Examples:
        save_image(xx, "123.png", "pallet")
        将保存在 static/images/pallet/123.png

    Args:
        file: 文件对象.
        filename: 文件名称.
        dir_name: 保存的目录名.

    Returns:
        url: 地址.
    """
    if not check_upload_image(file):
        raise XYZBaseError(
            error_message="Image file type unsupported, support type:[jpg, png]"
        )
    dir_name = dir_name or ""
    image_dir = os.path.join(settings.STATIC_FOLDER, "images", dir_name)
    save_file(save_dir=image_dir, file=file, filename=filename)
    return url_for("static", filename="/".join(["images", dir_name, filename]))


def remove_image(image_url: str):
    """删除静态目录下的图片文件.

    Args:
        image_url: 图片链接，例如： /static/images/1.png
    """
    image_path = image_url.replace("/static/", "")
    image_dir = os.path.join(settings.STATIC_FOLDER, image_path)
    os.remove(image_dir)


def register_upload(app):
    app.add_url_rule(
        '/uploads/<name>',
        endpoint='download_file',
        view_func=download_file
    )
