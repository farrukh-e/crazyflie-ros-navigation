from setuptools import find_packages, setup

package_name = 'crazyflie_ros2_test'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        (
            'share/ament_index/resource_index/packages',
            ['resource/' + package_name],
        ),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='FE',
    maintainer_email='argonaut@todo.todo',
    description='Test code package for Crazyflie ROS 2 experiments',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'test_node = crazyflie_ros2_test.test_node:main',
            'service_compat = crazyflie_ros2_test.service_compat:main',
        ],
    },
)
