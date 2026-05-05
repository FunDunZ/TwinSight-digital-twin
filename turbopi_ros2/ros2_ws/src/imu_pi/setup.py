from setuptools import find_packages, setup

package_name = 'imu_pi'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/imu_pi']),
        ('share/imu_pi', ['package.xml']),
        ('share/imu_pi/config', ['config/imu_config.json']),
    ],
    install_requires=['setuptools', 'smbus2'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@robot.local',
    description='BNO055 IMU publisher for ROS2',
    license='MIT',
    entry_points={
        'console_scripts': [
            'bno055_publisher = imu_pi.publish_imu_ros2:main',
        ],
    },
)
