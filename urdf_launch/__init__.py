"""
Launch extensions for URDF operations.

The launch frontends import this package by the entry point launch.frontend.launch_extension (setup.cfg), which
registers the substitutions for YAML / XML launch files. Python launch files use the classes directly.
"""

from .xacro_file import XacroFile

__all__ = ['XacroFile']
