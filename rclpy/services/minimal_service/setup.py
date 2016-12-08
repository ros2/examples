from setuptools import setup

setup(
    name='examples_rclpy_minimal_service',
    version='0.0.0',
    packages=[],
    py_modules=[
        'service',
        'service_member_function'],
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
            'examples_rclpy_minimal_service = service:main',
            'examples_rclpy_minimal_service_member_function = service_member_function:main',
        ],
    },
)
