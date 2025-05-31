from setuptools import setup
import os
from glob import glob

package_name = 'aruco_charger_navigation'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your.email@example.com',
    description='AruCo marker based navigation for autonomous docking',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'aruco_detector = aruco_charger_navigation.aruco_detector:main',
            'charger_navigator = aruco_charger_navigation.charger_navigator:main',
        ],
    },
)
