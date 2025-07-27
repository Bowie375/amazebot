import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'navigation'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'models'), glob('models/**')),
        (os.path.join('share', package_name, 'worlds'), glob('worlds/**')),
        (os.path.join('share', package_name, 'config'), glob('config/**')),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/**')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/**')),
        (os.path.join('share', package_name, 'maps'), glob('maps/**')),
        (os.path.join('share', package_name, package_name), glob(f'{package_name}/**')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='bowie',
    maintainer_email='2200013174@stu.pku.edu.cn',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'sensor_tf_publisher = navigation.sensor_tf_publisher:main',
            'odom_tf_publisher = navigation.odom_tf_publisher:main',
            'cmd_vel_publisher = navigation.cmd_vel_publisher:main',
            'start_kimi = navigation.start_kimi:main',
            'start_microphone = navigation.start_microphone:main',
        ],
    },
)
