from setuptools import setup

package_name = 'examples_rclpy_minimal_action_server'

setup(
    name=package_name,
    version='0.9.4',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    author='Jacob Perron',
    author_email='jacob@openrobotics.org',
    maintainer='Shane Loretz',
    maintainer_email='sloretz@openrobotics.org',
    keywords=['ROS'],
    classifiers=[
        'Intended Audience :: Developers',
        'License :: OSI Approved :: Apache Software License',
        'Programming Language :: Python',
        'Topic :: Software Development',
    ],
    description='Examples of action servers using rclpy.',
    license='Apache License, Version 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'server = ' + package_name + '.server:main',
            'server_defer = ' + package_name + '.server_defer:main',
            'server_not_composable = ' + package_name + '.server_not_composable:main',
            'server_queue_goals = ' + package_name + '.server_queue_goals:main',
            'server_single_goal = ' + package_name + '.server_single_goal:main',
        ],
    },
)
