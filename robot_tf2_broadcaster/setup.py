import os 
from glob import glob
from setuptools import find_packages, setup

package_name = 'robot_tf2_broadcaster'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*')))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='javier',
    maintainer_email='jlozanosoto@b2h.harvard.edu',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'robot_tf2_broadcaster = robot_tf2_broadcaster.robot_tf2_broadcaster:main'
        ],
    },
)
