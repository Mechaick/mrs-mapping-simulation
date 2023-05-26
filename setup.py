from setuptools import setup
from glob import glob
import os
package_name = 'mrs_mapping_simulation'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch/', glob('launch/*launch.[pxy][yma]*')),
        ('share/' + package_name + '/robot/', glob('robot/*')),
        ('share/' + package_name + '/config/', glob('config/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='lchovet',
    maintainer_email='loick.chovet@uni.lu',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'relative_tf = mrs_mapping_simulation.relative_tf:main',
            'tf_lookup = mrs_mapping_simulation.tf_lookup:main',
            'lidar_limiter = mrs_mapping_simulation.lidar_limiter:main',
            'relative_tf_tb = mrs_mapping_simulation.relative_tf_slam_tb:main',
        ],
    },
)
