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
    maintainer='hyunseo',
    maintainer_email='hyunseo@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'p1_1_follower = smyd.p1_1_follower:main',
            'p1_2_follower_cav1 = smyd.p1_2_follower_cav1:main',
            'p1_2_follower_cav2 = smyd.p1_2_follower_cav2:main',
            'zone_generator = smyd.tools.zone_generator:main',
        ],
    },
)
