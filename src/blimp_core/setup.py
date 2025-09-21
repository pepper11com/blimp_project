from setuptools import find_packages, setup

package_name = 'blimp_core'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # install launch files
        ('share/' + package_name + '/launch', ['launch/blimp_localization_launch.py',
                                                'launch/blimp_slam_launch.py']),
    ],
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
        ],
    },
)
