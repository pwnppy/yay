from setuptools import setup

package_name = 'imu_yaw_visualization'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='your_email@example.com',
    description='A package to visualize yaw angle from IMU quaternion',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'imu_yaw_visualizer = imu_yaw_visualization.imu_yaw_visualizer:main',
        ],
    },
)
