"""Resolve Python dependencies already installed with PlatformIO.

The desktop GUI uses the MSYS2 Python because it includes tkinter. PlatformIO
uses a separate Python environment that already includes pyserial. This module
adds that environment as a fallback only when the active Python cannot import
pyserial itself.
"""

from __future__ import annotations

import importlib
import os
from pathlib import Path
import sys
from typing import Iterable


def _platformio_site_packages() -> Iterable[Path]:
    configured_core = os.environ.get("PLATFORMIO_CORE_DIR")
    core_dir = Path(configured_core).expanduser() if configured_core else Path.home() / ".platformio"
    penv_dir = core_dir / "penv"

    # PlatformIO's standard Windows virtual environment layout.
    yield penv_dir / "Lib" / "site-packages"

    # Also support the layout used by PlatformIO on POSIX systems.
    lib_dir = penv_dir / "lib"
    if lib_dir.is_dir():
        yield from lib_dir.glob("python*/site-packages")


def ensure_pyserial_available() -> bool:
    """Make pyserial importable, preferring the active Python environment."""
    try:
        importlib.import_module("serial")
        return True
    except ImportError:
        pass

    for site_packages in _platformio_site_packages():
        if not (site_packages / "serial" / "__init__.py").is_file():
            continue
        site_packages_text = str(site_packages)
        if site_packages_text not in sys.path:
            # Append so packages installed in the active interpreter keep priority.
            sys.path.append(site_packages_text)
        importlib.invalidate_caches()
        try:
            importlib.import_module("serial")
            return True
        except ImportError:
            continue

    return False
