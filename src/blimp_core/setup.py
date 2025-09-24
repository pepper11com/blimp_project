from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'blimp_core'

# collect all files in the launch directory
launch_files = [f for f in glob(os.path.join('launch', '*')) if os.path.isfile(f)]

data_files = [
    ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
    ('share/' + package_name, ['package.xml']),
]
if launch_files:
    data_files.append(('share/' + package_name + '/launch', launch_files))

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=data_files,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='blimp',
    maintainer_email='blimp@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'msp_diag = blimp_core.msp_diag:main',
            'odom_to_path = blimp_core.odom_to_path:main',
            'blimp_obstacle_visualizer = blimp_core.blimp_obstacle_visualizer:main',
        ],
    },
)
