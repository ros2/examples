from setuptools import setup

setup(
    name='examples_rclpy_minimal_publisher',
    version='0.0.0',
    packages=[],
    py_modules=[
        'minimal_publisher',
        'minimal_publisher_timer_lambda',
        'minimal_publisher_class'],
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
    description='Examples of minimal publishers using rclpy.',
    license='Apache License, Version 2.0',
    test_suite='test',
    entry_points={
        'console_scripts': [
            'minimal_publisher_py = minimal_publisher:main',
            'minimal_publisher_timer_lambda_py = minimal_publisher_timer_lambda:main',
            'minimal_publisher_class_py = minimal_publisher_class:main',
        ],
    },
)
