"""Hardware project descriptions; independent of simulation runtime and UI."""

from .project import RobotProject, load_project
from .validation import check_project

__all__ = ["RobotProject", "load_project", "check_project"]
