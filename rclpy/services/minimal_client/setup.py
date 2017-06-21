from ament_python.script_dir import install_scripts_to_libexec
from setuptools import setup

package_name = 'examples_rclpy_minimal_client'
install_scripts_to_libexec(package_name)

setup(
    name=package_name,
    version='0.0.0',
    packages=[],
    py_modules=[
        'client',
        'client_async',
        'client_async_member_function'],
    install_requires=['setuptools'],
    author='Mikael Arguedas',
    author_email='Mikael@osrfoundation.org',
    maintainer='Mikael Arguedas',
    maintainer_email='mikael@osrfoundation.org',
    keywords=['ROS'],
    classifiers=[
        'Intended Audience :: Developers',
        'License :: OSI Approved :: Apache Software License',
        'Programming Language :: Python',
        'Topic :: Software Development',
    ],
    description='Examples of minimal service clients using rclpy.',
    license='Apache License, Version 2.0',
    test_suite='test',
    entry_points={
        'console_scripts': [
            'examples_rclpy_minimal_client = client:main',
            'examples_rclpy_minimal_client_async = client_async:main',
            'examples_rclpy_minimal_client_async_member_function ='
            ' client_async_member_function:main',
        ],
    },
)
