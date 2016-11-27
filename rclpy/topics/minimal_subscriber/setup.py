from setuptools import setup

setup(
    name='examples_rclpy_minimal_subscriber',
    version='0.0.0',
    packages=[],
    py_modules=[
        'minimal_subscriber', 'minimal_subscriber_lambda',
        'minimal_subscriber_local_function'],
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
    description='Examples of minimal subscribers using rclpy.',
    license='Apache License, Version 2.0',
    test_suite='test',
    entry_points={
        'console_scripts': [
            'minimal_subscriber_py = minimal_subscriber:main',
            'minimal_subscriber_lambda_py = minimal_subscriber_lambda:main',
            'minimal_subscriber_local_function_py = minimal_subscriber_local_function:main',
        ],
    },
)
