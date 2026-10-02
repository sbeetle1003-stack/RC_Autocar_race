from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'ultra_drive'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # launch file
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='root@todo.todo',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ultra_gostop = ultra_drive.ultra_gostop:main',
            'my_ultra_drive = ultra_drive.my_ultra_drive:main',
            'ultra_drive = ultra_drive.ultra_drive:main',
            'ultra_filter_drive = ultra_drive.ultra_filter_drive:main',
            'steering_viewer = ultra_drive.steering_viewer:main',
            'steering_viewer_arrow = ultra_drive.steering_viewer_arrow:main',
        ],
    },
)
