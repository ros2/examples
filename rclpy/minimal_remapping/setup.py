from setuptools import setup

package_name = 'examples_rclpy_minimal_remapping'

setup(
    name=package_name,
    version='0.4.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    author='Shane Loretz',
    author_email='sloretz@openrobotics.org',
    maintainer='Shane Loretz',
    maintainer_email='sloretz@openrobotics.org',
    keywords=['ROS'],
    classifiers=[
        'Intended Audience :: Developers',
        'License :: OSI Approved :: Apache Software License',
        'Programming Language :: Python',
        'Topic :: Software Development',
    ],
    description='Examples of creating and using remapping rules in rclpy.',
    license='Apache License, Version 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'init = examples_rclpy_minimal_remapping.init:main',
            'create_node = examples_rclpy_minimal_remapping.create_node:main',
            'node = examples_rclpy_minimal_remapping.node:main',
        ],
    },
)
