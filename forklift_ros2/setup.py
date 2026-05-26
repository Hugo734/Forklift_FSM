from setuptools import find_packages, setup

package_name = 'forklift_ros2'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'spidev'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='you@example.com',
    description='ROS2 driver for the forklift FPGA motor controller',
    license='MIT',
    entry_points={
        'console_scripts': [
            'forklift_driver = forklift_ros2.forklift_driver:main',
        ],
    },
)
