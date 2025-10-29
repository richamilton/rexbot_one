from glob import glob
from setuptools import setup

package_name = 'rexbot_one'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Include service files
        ('share/' + package_name + '/srv', glob('srv/*.srv')),
        # Include config files
        ('share/' + package_name + '/config', glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='you@example.com',
    description='Multi-floor robot navigation',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'delivery_coordinator = rexbot_one.delivery_coordinator:main',
            'map_manager = rexbot_one.map_manager:main',
            'nav_manager = rexbot_one.nav_manager:main',
            'set_initial_pose_once = rexbot_one.set_initial_pose_once:main'
        ],
    },
)