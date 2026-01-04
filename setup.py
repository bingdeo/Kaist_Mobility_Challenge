from setuptools import setup
from glob import glob

package_name = 'smyd'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', ['config/v2v_bridge.yaml']),
        ('share/' + package_name + '/waypoints', glob('waypoints/*.json')),
        ('share/' + package_name + '/tools', [
            'smyd/tools/zone_database.json'
            ]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='',
    maintainer_email='@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'p1_1 = smyd.p1_1:main',
            'p1_2_cav1 = smyd.p1_2_cav1:main',
            'p1_2_cav2 = smyd.p1_2_cav2:main',
            'zone_generator = smyd.tools.zone_generator:main',
        ],
    },
)
