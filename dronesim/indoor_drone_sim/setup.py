from setuptools import find_packages, setup
from glob import glob

package_name = 'indoor_drone_sim'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/rviz', glob('rviz/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='small',
    maintainer_email='small@todo.todo',
    description='Minimal indoor drone simulation with LiDAR, Geiger sensor and SLAM-ready topics',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'sim_world_node = indoor_drone_sim.sim_world_node:main',
            'sim_server_node = indoor_drone_sim.sim_server_node:main',
            'radmapper_node = indoor_drone_sim.radmapper:main',
            'radmapper_demo = indoor_drone_sim.radmapper:demo_main',
            'toucher_node = indoor_drone_sim.toucher_node:main',
        ],
    },
)
