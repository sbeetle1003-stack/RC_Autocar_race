from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'my_traffic'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # launch 파일을 설치
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='xytron',
    maintainer_email='xytron@todo.todo',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'stopline_check = my_traffic.stopline_check:main',
            'stopline_drive = my_traffic.stopline_drive:main',
            'trafficlight_check = my_traffic.trafficlight_check:main',
            'trafficlight_drive = my_traffic.trafficlight_drive:main',
            'trafficlight_detect = my_traffic.trafficlight_detect:main',
            'my_tflight = my_traffic.my_tflight:main',
            'my_tflight_image = my_traffic.my_tflight_image:main',
        ],
    },
)
