from glob import glob

from setuptools import setup

package_name = 'swarm_hack'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Asad',
    maintainer_email='smahrizvi64@gmail.com',
    description='TurtleBot SLAM + Crazyflie Flow-deck swarm: formation following and object inspection',
    license='MIT',
    entry_points={
        'console_scripts': [
            'crazyflie_bridge = swarm_hack.crazyflie_bridge:main',
            'swarm_coordinator = swarm_hack.swarm_coordinator:main',
            'object_mapper = swarm_hack.object_mapper:main',
            'video_recorder = swarm_hack.video_recorder:main',
        ],
    },
)
