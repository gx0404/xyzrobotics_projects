import io
import os
import typing
from typing import Any

from flask import Response, send_file

from apps.enums import MediaType


class StreamingResponse(Response):

    def __init__(
        self,
        stream: typing.IO,
        filename: str,
        media_type: typing.Union[str, MediaType] = None,
        as_attachment=False,
        add_etags=True,
        cache_timeout=None,
        conditional=False,
        last_modified=None,
        *args,
        **kwargs,
    ) -> None:
        self.__media_type = media_type.value if isinstance(
            media_type,
            MediaType
        ) else media_type
        self.__stream = stream
        self.__filename = filename
        self.__add_etags = add_etags
        self.__as_attachment = as_attachment
        self.__cache_timeout = cache_timeout
        self.__conditional = conditional
        self.__last_modified = last_modified
        super().__init__(*args, **kwargs)

    def __call__(self, *args, **kwargs):
        rv = send_file(
            filename_or_fp=self.__stream,
            mimetype=self.__media_type,
            as_attachment=self.__as_attachment,
            attachment_filename=self.__filename,
            add_etags=self.__add_etags,
            cache_timeout=self.__cache_timeout,
            conditional=self.__conditional,
            last_modified=self.last_modified,
        )

        self.response = rv.response
        self.headers.extend(rv.headers)
        self.mimetype = self.__media_type
        self.direct_passthrough = True
        self.last_modified = rv.last_modified
        self.expires = rv.expires
        self.cache_control.public = rv.cache_control.max_age
        self.cache_control.max_age = rv.cache_control.max_age
        self.headers["Access-Control-Allow-Origin"] = "*"
        self.headers["Access-Control-Allow-Headers"] = "*"
        self.headers["Access-Control-Expose-Headers"] = "Content-Disposition"
        return super(StreamingResponse, self).__call__(*args, **kwargs)


class FileResponse(Response):
    """a file response."""

    def __init__(
        self,
        path: typing.Union[str, "os.PathLike[str]"],
        media_type: typing.Union[str, MediaType] = None,
        filename: str = None,
        as_attachment=False,
        add_etags=True,
        cache_timeout=None,
        conditional=False,
        last_modified=None,
        *args,
        **kwargs,
    ):
        """
        Args:
            path: 文件路径.
            media_type: content-type
            filename: 文件名，如果不指定则截取path中的文件名.
            as_attachment: Default: False. 是否作为附件下载.
            add_etags:
            cache_timeout: 缓存时间, 在缓存时间内多次访问，将直接从内存中返回数据.
            conditional:
            last_modified: 上次修改时间
            *args:
            **kwargs:

        Examples:
            from apps import create_app
            from apps.enums import MediaType

            app = create_app()

            @app.route("/download")
            def main():
                return FileResponse(
                    path="/path/to/static/excel/demo.xlsx",
                    media_type=MediaType.xlsx,
                    as_attachment=True,
                )

            We will download the file from the server.

            @app.route("/show_image")
            def main():
                return FileResponse(
                    path="/path/to/static/image/1.png",
                    media_type=MediaType.png
                )

            We will look the image.
        """
        self.__path = path
        self.__media_type = media_type.value if isinstance(
            media_type,
            MediaType
        ) else media_type
        self.__filename = filename or path.rsplit("/", 1)[-1]
        self.__add_etags = add_etags
        self.__as_attachment = as_attachment
        self.__cache_timeout = cache_timeout
        self.__conditional = conditional
        self.__last_modified = last_modified
        super().__init__(*args, **kwargs)

    def __call__(self, *args, **kwargs):
        with open(self.__path, 'rb') as f:
            content = f.read()

        rv = send_file(
            filename_or_fp=io.BytesIO(content),
            mimetype=self.__media_type,
            as_attachment=self.__as_attachment,
            attachment_filename=self.__filename or self.__path.rsplit(".", 1)[
                -1],
            add_etags=self.__add_etags,
            cache_timeout=self.__cache_timeout,
            conditional=self.__conditional,
            last_modified=self.last_modified,
        )

        self.response = rv.response
        self.headers = rv.headers
        self.mimetype = self.__media_type
        self.direct_passthrough = True
        self.last_modified = rv.last_modified
        self.expires = rv.expires
        self.cache_control.public = rv.cache_control.max_age
        self.cache_control.max_age = rv.cache_control.max_age
        self.headers["Access-Control-Allow-Origin"] = "*"
        self.headers["Access-Control-Allow-Headers"] = "*"
        self.headers["Access-Control-Expose-Headers"] = "Content-Disposition"
        return super(FileResponse, self).__call__(*args, **kwargs)


class JSONResponse(Response):
    """a JSON response.

    Examples:

        from apps import create_app

        app = create_app()

        @app.route('/')
        def main():
            return JSONResponse(data={"code": 200, "message": "OK"})

        We will get a response:
            {
                "code": 200,
                "message": "OK",
            }
    """
    default_mimetype = MediaType.json.value

    def __init__(self, response: dict, *args, **kwargs):
        if isinstance(response, dict):
            response = self.json_module.dumps(response)  # type: ignore
        else:
            raise TypeError("Response must be a dictionary.")
        super(JSONResponse, self).__init__(response=response, *args, **kwargs)


class StdResponse(JSONResponse):
    """a standard response.

    Examples:

        from apps import create_app

        app = create_app()

        @app.route('/api/')
        def main():
            return StdResponse(data={}, extra_data={"abc": 123})

        We will get the response:
            {
                "code": 0,
                "msg": "success",
                "data": {},
                "abc": 123
            }
    """

    def __init__(
        self,
        code: int = 0,
        message: str = None,
        data: Any = None,
        extra_data: dict = None,
        **kwargs
    ):
        if message is None:
            message = "success" if code == 0 else "fail"
        response = {
            "code": code,
            "msg": message,
            "data": data
        }
        if isinstance(extra_data, dict):
            raise TypeError("extra_data must be a dictionary.")
        if extra_data is not None:
            response |= extra_data  # response.update(extra_data)
        super(StdResponse, self).__init__(response=response, **kwargs)
