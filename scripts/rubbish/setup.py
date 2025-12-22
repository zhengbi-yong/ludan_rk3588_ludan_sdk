from setuptools import setup
import os
from glob import glob

package_name = 'dds_lowcmd_converter'

setup(
    name=package_name,
    version='1.0.0',
    packages=[],
    py_modules=[],
    data_files=[
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        ('lib/' + package_name, ['src/dds_to_ros2_converter.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@example.com',
    description='DDS xixiLowCmd to ROS2 converter for Foxglove',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'dds_lowcmd_converter = dds_to_ros2_converter:main',
        ],
    },
    python_requires='>=3.6',
)