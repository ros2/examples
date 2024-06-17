from setuptools import setup

package_name = 'examples_rclpy_pointcloud_publisher'

setup(
    name=package_name,
    version='0.20.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Aditya Pande, Alejandro Hernandez Cordero',
    maintainer_email='aditya.pande@openrobotics.org, alejandro@openrobotics.org',
    description='Example on how to publish a Pointcloud2 message',
    license='Apache 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'pointcloud_publisher = examples_rclpy_pointcloud_publisher.pointcloud_publisher:main'
        ],
    },
)
