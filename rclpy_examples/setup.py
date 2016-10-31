from setuptools import setup

from rclpy.impl import rmw_implementation_tools

rmw_impls = rmw_implementation_tools.get_rmw_implementations()

entry_points = [
    'listener_py = listener_py:main',
    'talker_py = talker_py:main'
]

for rmw_impl in rmw_impls:
    entry_points.append('talker_py__' + rmw_impl + ' = talker_py:main_for_rmw_impl.' + rmw_impl)
    entry_points.append(
        'listener_py__' + rmw_impl + ' = listener_py:main_for_rmw_impl.' + rmw_impl)

setup(
    name='rclpy_examples',
    version='0.0.0',
    packages=[],
    py_modules=['listener_py', 'talker_py'],
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
        'console_scripts': entry_points,
    },
)
