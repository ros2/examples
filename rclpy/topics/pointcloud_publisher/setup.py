from setuptools import setup

package_name = 'examples_rclpy_pointcloud_publisher'

setup(
    name=package_name,
    version='0.11.2',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Evan Flynn',
    maintainer_email='evanflynn.msu@gmail.com',
    description='Example on how to publish a Pointcloud2 message',
    license='Apache 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'pointcloud_publisher = examples_rclpy_pointcloud_publisher.pointcloud_publisher:main'
        ],
    },
)
