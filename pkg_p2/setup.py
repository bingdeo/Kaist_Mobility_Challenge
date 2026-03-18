from setuptools import setup
import os
from glob import glob

package_name = 'pkg_p2'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),

        # launch 설치
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),

        # config 설치
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),

        # waypoints 설치
        (os.path.join('share', package_name, 'waypoints'), glob('waypoints/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='smyd',
    maintainer_email='inhsroy@hanyang.ac.kr',
    description='pkg_p2 controller',
    license='TODO',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'p2 = pkg_p2.p2:main',
        ],
    },
)
