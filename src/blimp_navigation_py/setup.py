from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'blimp_navigation_py'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'scripts'), glob('scripts/*.sh')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='blimp2',
    maintainer_email='blimp2@todo.todo',
    description='Pure Python package for blimp autonomous navigation',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'blimp_navigator = blimp_navigation_py.blimp_navigator:main',
        ],
    },
)