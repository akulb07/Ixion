"""Best-effort execution provenance without machine paths or environment secrets."""

import hashlib
import json
import platform
import subprocess
from datetime import UTC, datetime
from pathlib import Path

import numpy
import pydantic
import yaml

from roboforge import __version__


def source_identity(package: Path):
    """Fingerprint Python source on disk; this is not a capture of loaded bytecode."""
    source = {
        "scope": "roboforge Python source files on disk",
        "sha256": None,
        "files": {},
        "git_revision": None,
        "git_dirty": None,
        "notes": [],
    }
    try:
        files = sorted(package.rglob("*.py"))
        if (
            not files
            or len(files) > 1000
            or sum(p.stat().st_size for p in files) > 16 * 1024 * 1024
        ):
            raise ValueError("source inventory unavailable or exceeds budget")
        if any(p.is_symlink() for p in files):
            raise ValueError("source inventory contains symbolic links")
        source["files"] = {
            p.relative_to(package).as_posix(): hashlib.sha256(p.read_bytes()).hexdigest()
            for p in files
        }
        source["sha256"] = hashlib.sha256(
            json.dumps(source["files"], sort_keys=True, separators=(",", ":")).encode()
        ).hexdigest()
    except (OSError, ValueError):
        source["notes"].append("Python source fingerprint unavailable")
    # Only inspect the repository that contains this source checkout. Installed
    # wheels must not inherit an unrelated application's enclosing Git revision.
    root = package.parent.parent
    if package.parent.name == "src" and (root / ".git").exists():

        def git(*args):
            return subprocess.run(
                ["git", "-c", f"safe.directory={root.as_posix()}", "-C", str(root), *args],
                capture_output=True,
                text=True,
                timeout=2,
                check=True,
            ).stdout.strip()

        try:
            revision = git("rev-parse", "--verify", "HEAD")
            dirty = bool(git("status", "--porcelain", "--untracked-files=normal"))
            source.update(git_revision=revision, git_dirty=dirty)
        except (OSError, subprocess.SubprocessError):
            source["notes"].append("Git revision or working-tree state unavailable")
    return source


def capture_provenance():
    return {
        "format_version": 1,
        "captured_utc": datetime.now(UTC).isoformat(),
        "capture_stage": "service worker before simulation",
        "roboforge_version": __version__,
        "python": {
            "version": platform.python_version(),
            "implementation": platform.python_implementation(),
        },
        "platform": {"system": platform.system(), "machine": platform.machine()},
        "dependencies": {
            "numpy": numpy.__version__,
            "pydantic": pydantic.__version__,
            "PyYAML": yaml.__version__,
        },
        "source": source_identity(Path(__file__).resolve().parent),
    }
