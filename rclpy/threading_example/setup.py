from setuptools import setup

setup(
    name='threading_example',
    version='0.0.0',
    packages=[],
    py_modules=[
        'data_plotter',
        'data_publisher',
        ],
    install_requires=['setuptools'],
    keywords=['ROS'],
    classifiers=[
        'Intended Audience :: Developers',
        'License :: OSI Approved :: Apache Software License',
        'Programming Language :: Python',
        'Topic :: Software Development',
    ],
    description='Package containing examples of how to use the rclpy API.',
    license='Apache License, Version 2.0',
    entry_points={
        'console_scripts': [
            'data_plotter = data_plotter:main',
            'data_publisher = data_publisher:main',
        ],
    },
)
