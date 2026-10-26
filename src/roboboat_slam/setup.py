from setuptools import setup

package_name = 'roboboat_slam'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/slam_launch.py']),
        ('share/' + package_name + '/params', [
            'params/roboboat_localization.yaml',
            'params/slam_toolbox_params.yaml'
        ]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='phoenixautonomy',
    maintainer_email='your_email@example.com',
    description='Roboboat SLAM & filtering system',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'thruster_to_odom = roboboat_slam.thruster_to_odom:main'
        ],
    },
)
