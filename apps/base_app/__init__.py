import typing

if typing.TYPE_CHECKING:
    from apps import Application


def init_app(app: "Application"):
    """register blueprint or resource."""
    from .views.system.api import system_bp
    from .views.hmi.api import hmi_bp
    from .views.routes.api import bp as routes_bp
    from .views.error_records.api import bp as error_bp
    from .views.env.api import bp as env_bp
    from .views.topic.api import bp as topic_bp

    common_blueprints = [
        system_bp,
        hmi_bp,
        routes_bp,
        error_bp,
        env_bp,
        topic_bp,
    ]
    app.register_blueprints(common_blueprints)

    # register common resources
    from apps.base_app.views.command import (
        GetMode,
        GetStatus,
        Pause,
        RestartNode,
        Shutdown,
        Start,
        StartNode,
        Stop,
        UpdatePlcSignal
    )
    from apps.base_app.views.node.node_api import DownloadNodeLog, NodeLog
    from apps.base_app.views.notify import (
        NotifyNodeErr,
        NotifySystemLog,
        PublishOrderLog,
        TriggleWSEvent
    )
    from apps.base_app.views.query import (
        QueryAlive,
        QueryLog,
        QueryLogMenu,
        QueryNodesInfo,
        QueryRobotStatus
    )
    resources = [
        ((QueryAlive, "/"), {"ignore_repeat": True}),
        ((Start, "/api/cmd/start"), {"ignore_repeat": True}),
        ((Stop, "/api/cmd/stop"), {"ignore_repeat": True}),
        ((Pause, "/api/cmd/pause"), {"ignore_repeat": True}),
        ((Shutdown, "/api/cmd/shutdown"), {"ignore_repeat": True}),
        ((QueryNodesInfo, "/api/query/nodes_info"), {"ignore_repeat": True}),
        ((QueryLog, "/api/query/log"), {"ignore_repeat": True}),
        ((StartNode, "/api/cmd/start_node"), {"ignore_repeat": True}),
        ((RestartNode, "/api/cmd/restart_node"), {"ignore_repeat": True}),
        ((NotifyNodeErr, "/api/notify/node_error"), {"ignore_repeat": True}),
        ((NotifySystemLog, "/api/notify/system_log"), {"ignore_repeat": True}),
        ((QueryLogMenu, "/api/query/log_menu"), {"ignore_repeat": True}),
        ((QueryRobotStatus, "/api/query/robot_status"), {"ignore_repeat": True}),
        ((TriggleWSEvent, "/api/notify/websocket"), {"ignore_repeat": True}),
        ((PublishOrderLog, "/api/notify/send_order_log"), {"ignore_repeat": True}),
        ((DownloadNodeLog, "/api/download/log"), {"ignore_repeat": True}),
        ((NodeLog, "/api/query/node_log"), {"ignore_repeat": True}),
        ((UpdatePlcSignal, "/api/cmd/update_plc_signal"), {"ignore_repeat": True}),
        ((GetMode, "/api/cmd/get_mode"), {"ignore_repeat": True}),
        ((GetStatus, "/api/cmd/get_status"), {"ignore_repeat": True}),
    ]

    for resource, kwargs in resources:
        app.register_resource(*resource, **kwargs)
