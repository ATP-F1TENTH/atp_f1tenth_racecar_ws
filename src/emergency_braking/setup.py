from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'emergency_braking'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.xacro')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.rviz')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Sven Weinert',
    maintainer_email='sven.weinert@hsbi.de',
    description='Autonomatic Emergency Braking (AEB)',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'emergency_braking = emergency_braking.emergency_braking:main',
        ],
    },
)