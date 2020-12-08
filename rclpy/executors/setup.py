from setuptools import setup

package_name = 'examples_rclpy_executors'

setup(
    name=package_name,
    version='0.9.4',
    packages=['examples_rclpy_executors'],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
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
    description='Examples of creating and using exectors to run multiple nodes in rclpy.',
    license='Apache License, Version 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'listener = examples_rclpy_executors.listener:main',
            'talker = examples_rclpy_executors.talker:main',
            'callback_group = examples_rclpy_executors.callback_group:main',
            'composed = examples_rclpy_executors.composed:main',
            'custom_executor = examples_rclpy_executors.custom_executor:main',
            'custom_callback_group = examples_rclpy_executors.custom_callback_group:main',
        ],
    },
)
