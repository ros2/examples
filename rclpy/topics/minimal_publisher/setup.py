from setuptools import setup

setup(
    name='rclpy_examples',
    version='0.0.0',
    packages=[],
    py_modules=[
        'minimal_publisher', 'minimal_publisher_timer_lambda',
        'minimal_publisher_timer_local_function'],
    install_requires=['setuptools'],
    author='Esteve Fernandez',
    author_email='esteve@osrfoundation.org',
    maintainer='Esteve Fernandez',
    maintainer_email='esteve@osrfoundation.org',
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
            'minimal_publisher_py = minimal_publisher:main',
            'minimal_publisher_timer_lambda_py = minimal_publisher_timer_lambda:main',
            'minimal_publisher_timer_local_function_py = minimal_publisher_timer_local_function_py:main',
        ],
    },
)
