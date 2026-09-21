import importlib.metadata

import robot_math


def test_version_is_non_empty_string() -> None:
    assert isinstance(robot_math.__version__, str)
    assert robot_math.__version__ != ""


def test_installed_metadata_matches_package_version() -> None:
    assert importlib.metadata.version("robot_math") == robot_math.__version__
