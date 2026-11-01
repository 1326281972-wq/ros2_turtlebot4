import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'my_first_nodes'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob(os.path.join('launch', '*.launch.py'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='lud',
    maintainer_email='1326281972@qq.com',
    description='My ROS2 learning nodes: talker/listener, draw_square, add_server, turtle_to_point',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'my_talker = my_first_nodes.my_talker:main',
            'my_listener = my_first_nodes.my_listener:main',
            'draw_square = my_first_nodes.draw_square:main',
            'add_server = my_first_nodes.add_server:main',
            'turtle_to_point = my_first_nodes.turtle_to_point:main',
        ],
    },
)
