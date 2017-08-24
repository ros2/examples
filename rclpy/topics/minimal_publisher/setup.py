from setuptools import setup

package_name = 'examples_rclpy_minimal_publisher'

setup(
    name=package_name,
    version='0.0.0',
    packages=[],
    py_modules=[
        'publisher_old_school',
        'publisher_local_function',
        'publisher_member_function'],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    author='Mikael Arguedas',
    author_email='mikael@osrfoundation.org',
    maintainer='Mikael Arguedas',
    maintainer_email='mikael@osrfoundation.org',
    keywords=['ROS'],
    classifiers=[
        'Intended Audience :: Developers',
        'License :: OSI Approved :: Apache Software License',
        'Programming Language :: Python',
        'Topic :: Software Development',
    ],
    description='Examples of minimal publishers using rclpy.',
    license='Apache License, Version 2.0',
    test_suite='test',
    entry_points={
        'console_scripts': [
            'publisher_old_school = publisher_old_school:main',
            'publisher_local_function = publisher_local_function:main',
            'publisher_member_function = publisher_member_function:main',
        ],
    },
)
