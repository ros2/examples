from setuptools import setup

setup(
    name='examples_rclpy_minimal_client',
    version='0.0.0',
    packages=[],
    py_modules=[
        'minimal_client',
        'minimal_client_async'],
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
            'minimal_client_py = minimal_client:main',
            'minimal_client_async_py = minimal_client_async:main',
        ],
    },
)
