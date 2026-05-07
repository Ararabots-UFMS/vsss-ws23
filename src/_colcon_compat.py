"""Colcon compatibility for Python 3.14."""
from distutils.command.develop import develop as _develop


class DevelopCompat(_develop):
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
