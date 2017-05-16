from setuptools import setup

setup(
    name='examples_rclpy_generic_publisher',
    version='0.0.0',
    packages=[],
    py_modules=[
        'generic_publisher'],
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
            'generic_publisher_py = generic_publisher:main',
        ],
    },
)
