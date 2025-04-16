from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'act1_2'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ximena',
    maintainer_email='A01198557@tec.mx',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'movement = act1_2.movement:main',
            'vision = act1_2.vision:main',
            'ball = act1_2.ball:main',
            'pid_follower = act1_2.pid_follower:main'
        ],
    },
)
