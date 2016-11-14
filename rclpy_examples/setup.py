from setuptools import setup

setup(
    name='rclpy_examples',
    version='0.0.0',
    packages=[],
    py_modules=[
        'topics.listener_py', 'topics.talker_py',
        'topics.listener_qos_py', 'topics.talker_qos_py',
        'services.add_two_ints_client_py', 'services.add_two_ints_client_async_py',
        'services.add_two_ints_server_py'],
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
            'listener_py = topics.listener_py:main',
            'talker_py = topics.talker_py:main',
            'listener_qos_py = topics.listener_qos_py:main',
            'talker_qos_py = topics.talker_qos_py:main',
            'add_two_ints_client_py = services.add_two_ints_client_py:main',
            'add_two_ints_client_async_py = services.add_two_ints_client_async_py:main',
            'add_two_ints_server_py = services.add_two_ints_server_py:main'
        ],
    },
)
