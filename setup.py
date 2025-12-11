import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'sensor_fusion_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools', 'numpy'],
    zip_safe=True,
    maintainer='Kazuha Mogi',
    maintainer_email='mogi2fruits.kazu@gmail.com',
    description='1DKalmanFilter package',
    license='BSD-3-Clause',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'noisy_sensor = sensor_fusion_pkg.noisy_sensor:main',
            'fusion_node = sensor_fusion_pkg.fusion_node:main',
            'generic_kf_node = sensor_fusion_pkg.generic_kf_node:main',
        ],
    },
)
