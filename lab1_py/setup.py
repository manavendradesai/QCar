from setuptools import setup
import os
from glob import glob

package_name = 'lab1_py'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='manavendra',
    maintainer_email='manavendradesai@wayne.edu',
    description='Get familiarized with ROS 2 basics',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'talker = lab1_py.talker:main',
            'relay = lab1_py.relay:main',
        ],
    },
)
