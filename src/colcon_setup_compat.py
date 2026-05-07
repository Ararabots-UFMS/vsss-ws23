"""
Compatibility shim for colcon + Python 3.14.
Patches setuptools to accept --uninstall and --editable options in develop command.
"""
import sys
from distutils.command.develop import develop as _develop


class develop(_develop):
    """develop command that accepts colcon-specific options."""

    user_options = _develop.user_options + [
        ('uninstall', 'u', 'uninstall'),
        ('editable', 'e', 'editable'),
        ('build-directory=', None, 'build directory'),
    ]

    boolean_options = _develop.boolean_options + ['uninstall', 'editable']

    def initialize_options(self):
        super().initialize_options()
        self.uninstall = False
        self.editable = False
        self.build_directory = None

    def finalize_options(self):
        super().finalize_options()


def patch_setuptools():
    """Patch setuptools to use our compatible develop command."""
    try:
        from setuptools.command import develop as setuptools_develop
        setuptools_develop.develop = develop
    except ImportError:
        pass


# Auto-patch on import
patch_setuptools()
