from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'rse_shl1_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='carlos',
    maintainer_email='c.argueta@s1s2.ai',
    description='Swerve drive controller for SHL-1 robot',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'test_robot_control = rse_shl1_control.test_robot_control:main',
            'swerve_controller = rse_shl1_control.swerve_controller:main',
        ],
    },
)
