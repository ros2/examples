from setuptools import setup

package_name = 'examples_rclpy_minimal_timer'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Yadnyeshwar Sakhare',
    maintainer_email='yadnyeshwarasa@gmail.com',
    description='Minimal rclpy timer example demonstrating create_timer() usage.',
    license='Apache License 2.0',
    entry_points={
        'console_scripts': [
            'minimal_timer = examples_rclpy_minimal_timer.minimal_timer:main',
        ],
    },
)
