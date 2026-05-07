from setuptools.command.develop import develop as _develop
from setuptools import setup


class DevelopCompat(_develop):
    """develop command that accepts colcon-specific options."""
    user_options = _develop.user_options + [
        ('script-dir=', None, 'install scripts directory'),
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
        self.script_dir = None

    def finalize_options(self):
        super().finalize_options()

package_name = 'ble_message_server'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='marquesnavarezi@hotmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'MessageServerNode = ble_message_server.message_server_node:main',
        ],
    },
    cmdclass={'develop': DevelopCompat},
)
