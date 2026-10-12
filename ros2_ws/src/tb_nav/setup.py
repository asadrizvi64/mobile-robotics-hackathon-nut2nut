from glob import glob

from setuptools import setup

package_name = 'tb_nav'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml') + glob('config/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Asad',
    maintainer_email='smahrizvi64@gmail.com',
    description='TurtleBot SLAM, localization and Nav2 navigation manager',
    license='MIT',
    entry_points={'console_scripts': [
        'nav_manager = tb_nav.nav_manager:main',
        'obstacle_guard = tb_nav.obstacle_guard:main',
    ]},
)
