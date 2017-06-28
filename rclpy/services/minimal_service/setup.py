from ament_python.data_files import get_data_files
from ament_python.script_dir import install_scripts_to_libexec
from setuptools import setup

package_name = 'examples_rclpy_minimal_service'
data_files = get_data_files(package_name)
install_scripts_to_libexec(package_name)

setup(
    name=package_name,
    version='0.0.0',
    packages=[],
    py_modules=[
        'service',
        'service_member_function'],
    data_files=data_files,
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
    description='Examples of minimal service servers using rclpy.',
    license='Apache License, Version 2.0',
    test_suite='test',
    entry_points={
        'console_scripts': [
            'service = service:main',
            'service_member_function = service_member_function:main',
        ],
    },
)
