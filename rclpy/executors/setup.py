from ament_python.data_files import get_data_files
from ament_python.script_dir import install_scripts_to_libexec
from setuptools import setup

package_name = 'examples_rclpy_executors'
data_files = get_data_files(package_name)
install_scripts_to_libexec(package_name)

setup(
    name=package_name,
    version='0.0.2',
    packages=[],
    data_files=data_files,
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
    description='Examples of creating and using exectors to run multiple nodes in rclpy.',
    license='Apache License, Version 2.0',
    test_suite='test',
    entry_points={
        'console_scripts': [
            'listener = examples_executors.listener:main',
            'talker = examples_executors.listener:main',
            'composed = examples_executors.composed:main',
            'custom_executor = examples_executors.custom_executor:main',
        ],
    },
)
