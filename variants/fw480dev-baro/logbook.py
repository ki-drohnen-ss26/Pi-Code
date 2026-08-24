"""
logbook.py
==========
Central logging setup for the companion code. Configures one logger that writes
to BOTH the console and a timestamped file under ``log_dir/``, so every mission
run leaves a complete, reusable record for documentation and post-flight
analysis.

Call ``setup_logging()`` once at program start (see ``main.py``). Every module
obtains its logger via ``logging.getLogger(__name__)`` and its messages are
routed here automatically.
"""

from __future__ import annotations

import logging
from datetime import datetime
from pathlib import Path


def setup_logging(log_dir: str = "logs", level: int = logging.INFO) -> Path:
    """
    Configure console + file logging and return the path of the log file created.

    The file name carries a timestamp (``<log_dir>/mission_YYYYMMDD_HHMMSS.log``),
    so successive runs never overwrite each other. The console keeps the plain,
    tag-prefixed style the project already uses; the file additionally records a
    timestamp and the log level for later analysis.
    """
    directory = Path(log_dir)
    directory.mkdir(parents=True, exist_ok=True)
    log_file = directory / f"mission_{datetime.now():%Y%m%d_%H%M%S}.log"

    root = logging.getLogger()
    root.setLevel(level)

    # Drop handlers from a previous call (e.g. when tests configure logging
    # repeatedly) so messages are not duplicated.
    for handler in list(root.handlers):
        root.removeHandler(handler)

    file_handler = logging.FileHandler(log_file, encoding="utf-8")
    file_handler.setFormatter(
        logging.Formatter("%(asctime)s %(levelname)-7s %(name)s: %(message)s")
    )
    file_handler.setLevel(level)

    console_handler = logging.StreamHandler()
    console_handler.setFormatter(logging.Formatter("%(message)s"))
    console_handler.setLevel(level)

    root.addHandler(file_handler)
    root.addHandler(console_handler)

    logging.getLogger(__name__).info("[LOG] Logging to %s", log_file)
    return log_file
