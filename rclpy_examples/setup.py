from setuptools import setup

setup(
    name='rclpy_examples',
    version='0.0.0',
    packages=[],
    py_modules=['listener_py', 'talker_py', 'rosnode_py'],
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
            'listener_py = listener_py:main',
            'talker_py = talker_py:main',
            'rosnode_py = rosnode_py:main',
        ],
    },
)
