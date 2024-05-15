import os
import platform
import re
import shutil
import signal
import subprocess
import sys
import time
import typing
import warnings
from distutils.dir_util import copy_tree
from functools import update_wrapper
from operator import attrgetter
from pathlib import Path
from typing import Optional

import click
import flask
from click.core import Context
from flask_migrate import upgrade

from apps.log import hmi_log
from apps.models import migrate
from apps.utils.util import RobotStatePublisher

if typing.TYPE_CHECKING:
    from apps.app import Application


class ScriptInfo(object):

    def __init__(self):
        self._loaded_app = None

    def load_app(self) -> "Application":
        project_path = os.path.join("/home/xyz/xyz_app/app")
        back_path = Path(project_path, "xyz_logistics_hmi_back")
        app = load_app(back_path)
        self._loaded_app = app
        return app


def with_appcontext(f):
    """Wraps a callback so that it's guaranteed to be executed with the
    script's application context.  If callbacks are registered directly
    to the ``app.cli`` object then they are wrapped with this function
    by default unless it's disabled.
    """

    @click.pass_context
    def decorator(__ctx, *args, **kwargs):
        with __ctx.ensure_object(ScriptInfo).load_app().app_context():
            return __ctx.invoke(f, *args, **kwargs)

    return update_wrapper(decorator, f)


def load_app(back_path: Path) -> "Application":
    """创建app实例并加载wcs_adaptor.

    Args:
        back_path: 后端项目路径

    Returns:
        app: an instance of Application.
    """
    from apps import create_app
    app = create_app()
    # automatic upgrade db
    with app.app_context():
        directory = os.path.join(
            Path(os.path.realpath(__file__)).parent,
            "migrations"
        )
        upgrade(directory=directory)

    if os.getenv("XLHB_DEV") == "1":
        back_path = Path(os.path.abspath(__file__)).parent.parent

    # 加载wcs_adaptor.
    sys.path.insert(0, str(back_path))
    import wcs_adaptor
    # init wcs adaptor.
    wcs_adaptor.app.init_app(app)
    return app


def get_version() -> Optional[str]:
    """获取版本号."""
    from apps import settings
    try:
        from wcs_adaptor import version
        wcs_adaptor_version = version.__version__
    except ImportError:
        wcs_adaptor_version = "unknown"

    def fill(text: str, width: int) -> str:
        return text + " " * (width - len(text))

    python_version = fill(platform.python_version(), width=10)
    flask_version = fill(flask.__version__, width=10)
    xlhb_version = fill(settings.VERSION, width=10)
    wcs_adaptor_version = fill(wcs_adaptor_version, width=10)
    content = """
+-----------------------------+
| Package         | Version   |
+-----------------------------+
| Python          | {}|
| Flask           | {}|
| XLHB            | {}|
| WCS Adaptor     | {}|
+-----------------------------+
""".format(python_version, flask_version, xlhb_version, wcs_adaptor_version)

    return content


@click.group(invoke_without_command=True)
@click.version_option(get_version())
@click.pass_context
def cli(ctx: Context):
    """XLHB Command Line Tool.
    """
    ignore_commands = {"startproject", "dev"}
    # 重写直接运行xlhb的结果，默认为是显示帮助信息, 重写为运行程序.
    # xlhb -> xlhb run
    # python main_app.py -> python main_app.py run
    if ctx.invoked_subcommand is None:
        from apps import create_app
        # 判断当前环境是否为开发环境
        flask_env = (os.getenv("FLASK_ENV") or "").lower()
        _isolate_database(app=create_app(), is_dev=flask_env == "development")

        # 判断main_app运行目录是否存在wcs_adaptor与config
        # 如果有, 则加载。用于在开发apps过程中，直接运行main_app.py时，可直接加载同目录下的wcs_adaptor与config
        wcs_adaptor = config = None
        if len(sys.argv) == 1:
            main_app_path = Path(sys.argv[0])
            run_path = main_app_path.parent
            config_path = os.path.join(run_path, "config")
            wcs_adaptor_path = os.path.join(run_path, "wcs_adaptor")
            if os.path.exists(wcs_adaptor_path):
                wcs_adaptor = wcs_adaptor_path
            if os.path.exists(config_path):
                config = config_path
        ctx.invoke(run_command, wcs_adaptor=wcs_adaptor, config=config)
    elif ctx.invoked_subcommand not in ignore_commands:
        from apps import create_app
        # 判断当前环境是否为开发环境
        flask_env = (os.getenv("FLASK_ENV") or "").lower()
        _isolate_database(app=create_app(), is_dev=flask_env == "development")


robot_state_publisher = RobotStatePublisher()
main_pid = os.getpid()


def handler_signal(signum, frame):
    """处理结束信号.

    Args:
        signum: 信号值.
        frame: 调用栈.

    """
    from apps import hmi_log
    from apps.ext.offline_mix_planner import OfflineMixedTaskPlanner
    # 仅主进程才可调用
    if main_pid != os.getppid():
        hmi_log.info(f'Signal handler called with signal({signum})')
        if OfflineMixedTaskPlanner._instances:
            OfflineMixedTaskPlanner._instances.shutdown()
        robot_state_publisher.close()
    exit(signum)


version_option = click.Option(
    ["--version"],
    help="Show the XLHB version.",
    expose_value=False,
    callback=get_version,
    is_flag=True,
    is_eager=True,
)


@cli.command("dev", short_help="Enter development environment.")
def dev_command():
    from apps import settings
    os.environ["FLASK_ENV"] = "development"
    old_ps1 = os.environ["PS1"]
    os.environ["PS1"] = "\[\e[01;33m\][开发模式]\[\e[01;00m\](" \
                        + settings.PROJECT_NAME + ")" + old_ps1.split(")", 1)[1]
    click.echo(click.style(">> 已激活开发模式终端(仅用于后端开发), 键入Ctrl+d退出 <<", fg="yellow"))
    os.system("bash --noprofile --norc")


@cli.command("run", short_help="Run a local server.")
@click.option(
    "--config",
    "-c",
    type=click.Path(exists=True),
    help="Load specified configuration path."
)
@click.option(
    "--wcs-adaptor",
    "-a",
    type=click.Path(exists=True),
    help="Load specified the WCS Adaptor."
)
@click.option(
    "--project-name",
    "-p",
    help="Project name."
)
@click.option(
    "--debug",
    is_flag=True,
    help="Open debug mode."
)
def run_command(
    config: Optional[str],
    wcs_adaptor: Optional[str],
    project_name: Optional[str],
    debug: bool,
):
    """Run a local server."""
    from apps import settings

    back_path = _make_back_path(project_name, wcs_adaptor)

    app = load_app(back_path)

    _upgrade_version(back_path)

    # 如果启动了debug模式，会新增一个WERKZEUG_RUN_MAIN环境变量
    # 在debug模式中，会启用两个后端程序，此处用于判断当前程序是否为主程序
    # 只有主程序才去调用run_robot_state_publisher
    if not(settings.DEBUG and os.environ.get("WERKZEUG_RUN_MAIN")):
        robot_state_publisher.run()
        signal.signal(signalnum=signal.SIGINT, handler=handler_signal)
        signal.signal(signalnum=signal.SIGTERM, handler=handler_signal)
        signal.signal(signalnum=signal.SIGQUIT, handler=handler_signal)

    show_server_banner(settings, project_name, back_path)

    # debug: 通过命令行传入.
    # settings.DEBUG: 通过配置文件配置的.
    # 任意一个为True，即以debug模式运行.

    try:
        app.run_by_socketio(debug=debug or settings.DEBUG)
    except Exception as err:
        hmi_log.error(msg="后端程序启动失败", exc_info=True)
        handler_signal(signal.SIGTERM, err)


def _make_back_path(
    project_name: Optional[str],
    wcs_adaptor: Optional[str]
) -> Path:
    """生产后端目录路径.

    Args:
        project_name: 项目名称.
        wcs_adaptor: adaptor目录

    Returns:
        Path: 后端项目的路径.
    """
    if not project_name:
        project_path = os.path.join("/home/xyz/xyz_app/app")
    else:
        project_path = os.path.join("/home/xyz/xyz_app/projects", project_name)
    if not os.path.exists(project_path):
        raise RuntimeError("项目不存在，请检查项目名称是否正确.")
    if os.getenv("XLHB_DEV") == "1":
        back_path = Path(os.path.abspath(__file__)).parent.parent
    else:
        back_path = Path(project_path, "xyz_logistics_hmi_back")
    if wcs_adaptor:
        back_path = Path(wcs_adaptor).parent
    return back_path


def _upgrade_version(back_path: Path):
    """升级版本时，对wcs_adaptor目录做的一些操作.

    Args:
        back_path(pathlib.Path): 项目目录
    """
    # NOTICE:
    #   Version: 1.1.* -> 1.2.0
    #   判断wcs_adaptor/api/task_history/是否存在， 不存在则从模板目录自动复制
    template_path = "/home/xyz/xyz_app/xyz_logistics_hmi_back/"
    if not os.path.exists(
        os.path.join(back_path, "wcs_adaptor/api/task_history")
    ):
        copy_tree(
            os.path.join(template_path, "wcs_adaptor/api/task_history"),
            os.path.join(back_path, "wcs_adaptor/api/task_history")
        )
        with open(
            os.path.join(back_path, "wcs_adaptor/api/__init__.py"),
            "a"
        ) as f:
            f.write("from .task_history import *")
        warnings.warn(
            "V1.1.* -> V1.2.0版本变动: 新增历史任务下载功能, task_history目录缺失, 已自动复制到当前wcs_adaptor目录."
        )


def show_server_banner(settings, project_name, back_path):
    """程序启动时，展示概要信息.

    Args:
        settings(Settings): 配置对象.
        project_name(str): 项目名称.
        back_path(Path): 后端项目目录路径.
    """

    def _echo():
        try:
            from wcs_adaptor import version
            wcs_adaptor_version = version.__version__
        except ImportError:
            wcs_adaptor_version = "unknown"
        click.echo("===== XLHB Summary Info =====")
        click.echo(f" * Project name: {project_name or settings.PROJECT_NAME}")
        click.echo(f" * Project type: {settings.PROJECT_TYPE}")
        click.echo(f" * Config path: {settings.GENERAL_CONFIG_DIR}")
        click.echo(
            f" * WCS adaptor path: {os.path.join(back_path, 'wcs_adaptor')}"
        )
        click.echo(f" * Database uri: {settings.SQLALCHEMY_DATABASE_URI}")
        click.echo(f" * XLHB version: {settings.VERSION}")
        click.echo(f" * Adaptor version: {wcs_adaptor_version}")
        click.echo(f" * XLHB address: {settings.HTTP_ADDR}")
        click.echo(f" * WCS address: {settings.WCS_ADDR}")

    # 判断当前是否为debug子进程，子进程需要输出debugger已激活字样.
    if settings.DEBUG:
        if os.environ.get("WERKZEUG_RUN_MAIN") == "true":
            _echo()
            environment = os.environ.get("FLASK_ENV", "production")
            click.echo(
                click.style(f" * Environment: {environment}", fg='red')
            )
            click.echo(click.style(' * Debugger is active!', fg='red'))
    else:
        _echo()


def _isolate_database(app: "Application", is_dev: bool) -> str:
    """根据是否为开发环境隔离数据库.

    1. 自动创建数据库;
    2. 隔离开发环境与生产环境的数据库;

    Args:
        app: Application对象.
        is_dev: 是否为开发环境.

    Returns:
        str: 新数据库名.
    """
    from apps import settings, db
    from apps.settings import MySQLDsn

    uri, tail = settings.SQLALCHEMY_DATABASE_URI.rsplit("/", 1)
    db_name = re.search(r"(\w+)", tail).groups()[0]

    if is_dev:
        db_name += f"_{settings.PROJECT_NAME}"

    _create_database(db, db_name, uri)

    if is_dev:
        uri = MySQLDsn(
            url=f"{uri}/{db_name}",
            scheme=settings.SQLALCHEMY_DATABASE_URI.scheme,
            user=settings.SQLALCHEMY_DATABASE_URI.user,
            password=settings.SQLALCHEMY_DATABASE_URI.password,
            host=settings.SQLALCHEMY_DATABASE_URI.host,
            host_type=settings.SQLALCHEMY_DATABASE_URI.host_type,
            port=settings.SQLALCHEMY_DATABASE_URI.port,
            path=settings.SQLALCHEMY_DATABASE_URI.path,
            query=settings.SQLALCHEMY_DATABASE_URI.query,
            fragment=settings.SQLALCHEMY_DATABASE_URI.fragment,
        )
        settings.SQLALCHEMY_DATABASE_URI = uri
        app.config["SQLALCHEMY_DATABASE_URI"] = settings.SQLALCHEMY_DATABASE_URI
        db.init_app(app)
        migrate.init_app(app, db)
    return db_name


def _create_database(db, db_name, uri):
    """create database if not exists"""
    create_statement = f"CREATE DATABASE IF NOT EXISTS {db_name};"
    engine = db.create_engine(uri, {})
    engine.execute(create_statement)


@cli.command(
    "startproject",
    short_help="Generate a new project with template."
)
@click.option(
    "--project-type",
    "-t",
    type=click.Choice(["dpt", "pp", "ind"], case_sensitive=False),
    help="项目类型",
    prompt="请选择项目类型"
)
def startproject_command(project_type: str) -> None:
    # 检查项目模板目录是否存在
    template_path = "/home/xyz/xyz_app/xyz_logistics_hmi_back/"
    if not os.path.exists(template_path):
        raise FileNotFoundError("模板目录不存在, 请确认xyz-logistics-hmi-back已安装.")

    from apps import settings

    src_dir = template_path
    dest_dir = settings.Config.config_path.parent

    for file_name in os.listdir(src_dir):
        if file_name == "config":
            continue
        try:
            shutil.copytree(
                os.path.join(src_dir, file_name),
                os.path.join(dest_dir, file_name)
            )
        except FileExistsError as err:
            raise FileExistsError(f"生成失败, 目标文件已存在, {err.filename}") from None

    settings.PROJECT_TYPE = project_type
    settings.dumps()
    click.echo(f"项目类型[{project_type}]生成完成, 项目位于/home/xyz/xyz_app/app/xyz_logistics_hmi_back")


@cli.command(
    "shell",
    short_help="Run a shell in the app context.",
    context_settings=dict(ignore_unknown_options=True)
)
@click.argument('ipython_args', nargs=-1, type=click.UNPROCESSED)
@with_appcontext
def shell_command(ipython_args) -> None:
    """Runs a shell in the app context.

    Runs an interactive Python shell in the context of a given
    Flask application. The application will populate the default
    namespace of this shell according to its configuration.
    This is useful for executing small snippets of management code
    without having to manually configure the application.
    """
    import IPython
    from IPython.terminal.ipapp import load_default_config
    from traitlets.config.loader import Config
    from flask.globals import _app_ctx_stack

    app = _app_ctx_stack.top.app

    if 'IPYTHON_CONFIG' in app.config:
        config = Config(app.config['IPYTHON_CONFIG'])
    else:
        config = load_default_config()

    config.TerminalInteractiveShell.banner1 = '''Python %s on %s
    IPython: %s
    App: %s [%s]
    Instance: %s''' % (sys.version,
                       sys.platform,
                       IPython.__version__,
                       app.import_name,
                       app.env,
                       app.instance_path)

    config.InteractiveShellApp.exec_lines = [
        "import wcs_adaptor",
        "import apps"
    ]

    IPython.start_ipython(
        argv=ipython_args,
        user_ns=app.make_shell_context(),
        config=config,
    )


@cli.command("routes", short_help="Show the routes for the app.")
@click.option(
    "--sort",
    "-s",
    type=click.Choice(("endpoint", "methods", "rule", "match")),
    default="endpoint",
    help=(
        'Method to sort routes by. "match" is the order that Flask will match '
        "routes when dispatching a request."
    ),
)
@click.option(
    "--all-methods",
    is_flag=True,
    help="Show HEAD and OPTIONS methods."
)
@with_appcontext
def routes_command(sort: str, all_methods: bool) -> None:
    """Show all registered routes with endpoints and methods."""
    from flask import current_app

    endpoint_maps = {rule.endpoint: rule for rule in
                     current_app.url_map.iter_rules()}
    rules = endpoint_maps.values()

    if not rules:
        click.echo("No routes were registered.")
        return

    ignored_methods = set(() if all_methods else ("HEAD", "OPTIONS"))

    if sort in {"endpoint", "rule"}:
        rules = sorted(rules, key=attrgetter(sort))
    elif sort == "methods":
        rules = sorted(rules, key=lambda rule: sorted(rule.methods))

    rule_methods = [", ".join(sorted(rule.methods - ignored_methods)) for rule
                    in rules]

    headers = ("Endpoint", "Methods", "Rule")
    widths = (
        max(len(rule.endpoint) for rule in rules),
        max(len(methods) for methods in rule_methods),
        max(len(rule.rule) for rule in rules),
    )
    widths = [max(len(h), w) for h, w in zip(headers, widths)]
    row = "{{0:<{0}}}  {{1:<{1}}}  {{2:<{2}}}".format(*widths)

    click.echo(row.format(*headers).strip())
    click.echo(row.format(*("-" * width for width in widths)))

    for rule, methods in zip(rules, rule_methods):
        click.echo(row.format(rule.endpoint, methods, rule.rule).rstrip())


@cli.command("docs", short_help="View usage docs.")
@click.option(
    "--version",
    "-v",
    help="Open docs for a specific version."
)
def docs_command(version: str) -> None:
    """View usage docs.

    Default to the latest version.

    Please option --version to view docs for a specific version.
    """
    url = "https://161.189.84.82:8003/xyz-release-doc/ubuntu2004/hmi/"
    if version:
        url += version
    click.launch(url)
    click.echo("Docs was opened in browser.")


@cli.group("db", help="Manager HMI database.")
def db_cli() -> None:
    pass


@db_cli.command("upgrade", short_help="Upgrade database.")
@click.option(
    "--self",
    "-s",
    is_flag=True,
    help="Upgrade database of the XLHB."
)
@click.option(
    "--adaptor",
    "-a",
    is_flag=True,
    help="Upgrade database of the WCS adaptor."
)
def upgrade_db_command(self, adaptor):
    from apps import create_app, settings
    from flask_migrate import upgrade

    if not self and not adaptor:
        click.echo("Warning!!! Please muse be set -s/--self or -a/--adaptor.")
        return

    app = create_app()

    apps_path = Path(os.path.realpath(__file__)).parent
    back_path = settings.Config.config_path.parent

    # 在开发apps环境时, 使用main_app.py运行可直接加载同目录下的config与wcs_adaptor
    if sys.argv[0] == "main_app.py":
        run_path = Path(sys.argv[0]).parent
        config_path = os.path.join(run_path, "config")
        wcs_adaptor_path = os.path.join(run_path, "wcs_adaptor")
        if os.path.exists(wcs_adaptor_path) and os.path.exists(config_path):
            back_path = run_path
            apps_path = os.path.join(run_path, "apps")

    if self:
        click.echo("Upgrading database of the XLHB.")
        with app.app_context():
            upgrade(directory=os.path.join(apps_path, "migrations"))

    if adaptor:
        click.echo("Upgrading database of the WCS adaptor.")
        with app.app_context():
            upgrade(directory=os.path.join(back_path, "wcs_adaptor/migrations"))


@db_cli.command("downgrade", short_help="Downgrade database.")
@click.option(
    "--self",
    "-s",
    is_flag=True,
    help="Downgrade database of the XLHB."
)
@click.option(
    "--adaptor",
    "-a",
    is_flag=True,
    help="Downgrade database of the WCS adaptor."
)
def downgrade_db_command(self, adaptor):
    from apps import create_app, settings
    from flask_migrate import downgrade

    if not self and not adaptor:
        click.echo("Warning!!! Please muse be set -s/--self or -a/--adaptor.")
        return

    app = create_app()

    apps_path = Path(os.path.realpath(__file__)).parent
    back_path = settings.Config.config_path.parent

    # 在开发apps环境时, 使用main_app.py运行可直接加载同目录下的config与wcs_adaptor
    if sys.argv[0] == "main_app.py":
        run_path = Path(sys.argv[0]).parent
        config_path = os.path.join(run_path, "config")
        wcs_adaptor_path = os.path.join(run_path, "wcs_adaptor")
        if os.path.exists(wcs_adaptor_path) and os.path.exists(config_path):
            back_path = run_path
            apps_path = os.path.join(run_path, "apps")

    if self:
        app = create_app()
        click.echo("Downgrading database of the XLHB.")
        with app.app_context():
            downgrade(directory=os.path.join(apps_path, "migrations"))

    if adaptor:
        click.echo("Downgrading database of the WCS adaptor.")
        with app.app_context():
            downgrade(
                directory=os.path.join(back_path, "wcs_adaptor/migrations")
            )


@db_cli.command("migrate", short_help="Make migration file.")
@click.option(
    "--self",
    "-s",
    is_flag=True,
    show_default=True,
    help="Make migration file for the XLHB."
)
@click.option(
    "--message",
    "-m",
    help="a description for this commit."
)
def migrate_db_command(self, message: str = ""):
    """为wcs adaptor中的数据表结构生成versions"""
    from apps import create_app, settings
    from flask_migrate import migrate

    app = create_app()
    back_path = settings.Config.config_path.parent

    # 加载wcs_adaptor
    import wcs_adaptor
    _ = wcs_adaptor

    with app.app_context():
        if self:
            # 仅生成 apps 相关的迁移文件
            migrate(
                directory=os.path.join(back_path, "apps/migrations"),
                message=message
            )
            return

        migrate(
            directory=os.path.join(back_path, "wcs_adaptor/migrations"),
            message=message
        )


@db_cli.command("clear", short_help="Clear database.")
@click.option(
    "--skip-backup",
    is_flag=True
)
def clear_db_command(skip_backup):
    """清空HMI库中的所有表数据."""
    from apps import db, settings

    db_name = settings.SQLALCHEMY_DATABASE_URI \
        .rsplit("/", 1)[-1].split("?", 1)[0]
    user, passwd = settings.SQLALCHEMY_DATABASE_URI.user, settings.SQLALCHEMY_DATABASE_URI.password

    if db_name == "hmi":
        confirm = click.confirm(
            click.style("警告：即将删除正式环境hmi数据库, 请再次确认", fg="red"),
            abort=False
        )
        command = ["/usr/bin/mysqldump", f"--user={user}", f"-p{passwd}", db_name]
    else:
        confirm = click.confirm(
            click.style(f"注意：即将删除开发环境{db_name}数据库, 请再次确认", fg="yellow"),
            abort=False
        )
        command = ["/usr/bin/mysqldump", f"--user={user}", f"-p{passwd}", db_name]

    if not confirm:
        click.echo("* 操作取消")
        return

    def drop_database():
        create_statement = f"DROP DATABASE {db_name};"
        engine = db.create_engine(settings.SQLALCHEMY_DATABASE_URI, {})
        engine.execute(create_statement)

    if not skip_backup:
        _dir = Path(settings.GENERAL_CONFIG_DIR.parent, "db_backup").absolute()
        if not os.path.exists(_dir):
            os.mkdir(_dir)

        file_path = f"{_dir}/{db_name}_{int(time.time())}.sql"
        with open(file_path, "w") as f:
            with subprocess.Popen(
                command, 
                stdout=f, 
                stderr=subprocess.STDOUT,
                bufsize=0,
                env={"MYSQL_PWD": passwd},  # type: ignore
            ) as p:
                p.wait()
                if p.returncode == 0:
                    drop_database()
                    t = click.style(
                        f"* 已删除[{db_name}]数据库, SQL文件已备份至 {file_path}",
                        fg="green"
                    )
                else:
                    t = click.style(
                        f"* [{db_name}]数据库备份失败",
                        fg="red"
                    )
    else:
        drop_database()
        t = click.style(
            f"* 已删除[{db_name}]数据库",
            fg="green"
        )
    click.echo(t)


@db_cli.command("dump", short_help="Dumps database to file.")
def dump_db_command():
    """将数据库转存为 `sql` 文件."""
    from apps import settings

    db_name = settings.SQLALCHEMY_DATABASE_URI \
        .rsplit("/", 1)[-1].split("?", 1)[0]
    user, passwd = settings.SQLALCHEMY_DATABASE_URI.user, settings.SQLALCHEMY_DATABASE_URI.password
    _dir = Path(settings.GENERAL_CONFIG_DIR.parent, "db_backup").absolute()
    if not os.path.exists(_dir):
        os.mkdir(_dir)
    file_path = f"{_dir}/{db_name}_{int(time.time())}.sql"
    command = ["/usr/bin/mysqldump", f"--user={user}", db_name]
    with open(file_path, "w") as f:
        with subprocess.Popen(
            command,
            stdout=f,
            stderr=subprocess.STDOUT,
            bufsize=0,
            env={"MYSQL_PWD": passwd},  # type: ignore
        ) as p:
            p.wait()
            if p.returncode == 0:
                t = click.style(
                    f"SQL文件已备份至 {file_path}\n"
                    f"提示：如需还原可执行 xlhb db load [filename]",
                    fg="green"
                )
            else:
                t = click.style(
                    f"* [{db_name}]数据库备份失败",
                    fg="red"
                )
        click.echo(t)


@db_cli.command("load", short_help="Load data to database from sql file.")
@click.option(
    "--file",
    "-f",
    type=click.Path(exists=True),
)
def load_db_command(file: Path = None):
    """从 `sql` 文件还原数据到数据库."""
    from apps import settings

    db_name = settings.SQLALCHEMY_DATABASE_URI \
        .rsplit("/", 1)[-1].split("?", 1)[0]

    user, passwd = settings.SQLALCHEMY_DATABASE_URI.user, settings.SQLALCHEMY_DATABASE_URI.password
    if file is None:
        _dir = Path(settings.GENERAL_CONFIG_DIR.parent, "db_backup").absolute()
        # 让调用者确认：是否默认使用最新的 sql 文件
        # 可能存储多个 sql 文件, 如果没有做选择，则选择最新时间戳最近的文件.
        # [hmi_1661157585.sql, hmi_xlhb_dev_1661157686] -> [hmi_1661157585.sql]
        files = filter(lambda x: db_name in x, os.listdir(_dir))
        # hmi_1661157585.sql -> 1661157585.sql
        temps = map(lambda x: x.rsplit("_", 1)[-1], files)
        select_file = max(temps)
        file = os.path.join(_dir, f"{db_name}_{select_file}")
    elif not os.path.exists(file):
        click.echo("文件不存在, 请检查文件路径是否正确.")
        return

    r = click.prompt(f"即将使用 {file} 还原数据库\n请确认", default="y", type=click.Choice(["y", "n"]), show_default=True)
    if r.lower() != "y":
        click.echo("* 已取消")
        return

    command = ["mysql", "--defaults-file=/etc/mysql/my.cnf", "-u", user, f"--password={passwd}", db_name, "<", file]
    p = os.system(" ".join(command))
    if p == 0:
        t = click.style(
            f"{db_name} 数据已还原!",
            fg="green"
        )
    else:
        t = click.style("数据库还原失败", fg="red")
    click.echo(t)


def main() -> None:
    cli.main()


if __name__ == "__main__":
    main()
