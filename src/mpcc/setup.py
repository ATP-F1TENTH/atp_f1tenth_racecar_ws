from setuptools import find_packages, setup
from glob import glob
import os
package_name = 'mpcc'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
            (os.path.join('share', package_name, 'config'), glob('config/*.csv')),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='itse',
    maintainer_email='niklas.dohm@hsbi.de',
    description='Package implementing the model predictive control on f1tenth',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'mpcc = mpcc.mpcc:main'
        ],
    },
)
