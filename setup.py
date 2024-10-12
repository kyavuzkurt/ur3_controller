import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'ur3_controller'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'),
            glob('config/*.yaml')),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Kadir Yavuz Kurt',
    maintainer_email='k.yavuzkurt1@gmail.com',
    description='UR3 Controller',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ur_predefined_movement = ur3_controller.ur_predef_movement:main',
        ],
    },
)
