from contextlib import contextmanager
from typing import ContextManager, Iterator

from flask_migrate import Migrate
from flask_sqlalchemy import SQLAlchemy
from sqlalchemy.orm.scoping import ScopedSession

db = SQLAlchemy()
migrate = Migrate(db=db)

from .log import Log
from .routes import RouteDBModel


@contextmanager
def start_transaction() -> Iterator[ScopedSession]:
    """开启一个事务会话.

    Examples:
        This makes this:

            with start_transaction() as session:
                session.add(XXXModel(...))

        equivalent to this:

            from apps import db

            db.session.add(XXXModel(...))
            try:
                db.session.commit()
            except Exception:
                db.session.rollback()
    """
    try:
        yield db.session
        db.session.commit()
    except Exception as err:
        db.session.rollback()
        raise err
