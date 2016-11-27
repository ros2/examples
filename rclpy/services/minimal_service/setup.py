from setuptools import setup

setup(
    name='examples_rclpy_minimal_service',
    version='0.0.0',
    packages=[],
    py_modules=[
        'minimal_service',
        'minimal_service_local_function'],
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
    description='Package containing examples of how to use the rclpy API.',
    license='Apache License, Version 2.0',
    test_suite='test',
    entry_points={
        'console_scripts': [
            'minimal_service_py = minimal_service:main',
            'minimal_service_local_function_py = minimal_service_local_function:main',
        ],
    },
)
