from setuptools import setup

package_name = 'abv_dock_bridge'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='scott',
    maintainer_email='scott@todo.todo',
    description='Serial bridge to the docking mechanism Arduino, and the per-cycle docking test logger',
    license='TODO',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'dock_bridge = abv_dock_bridge.dock_bridge_node:main',
            'dock_logger = abv_dock_bridge.dock_logger_node:main',
        ],
    },
)
