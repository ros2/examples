from setuptools import setup

package_name = 'examples_rclpy_minimal_action_client'

setup(
    name=package_name,
    version='0.15.4',
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
    maintainer='Aditya Pande, Shane Loretz',
    maintainer_email='aditya.pande@openrobotics.org, shane@openrobotics.org',
    keywords=['ROS'],
    classifiers=[
        'Intended Audience :: Developers',
        'Programming Language :: Python',
        'Topic :: Software Development',
    ],
    description='Examples of action clients using rclpy.',
    license='Apache License, Version 2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'client = ' + package_name + '.client:main',
            'client_cancel = ' + package_name + '.client_cancel:main',
            'client_not_composable = ' + package_name + '.client_not_composable:main',
            'client_asyncio = ' + package_name + '.client_asyncio:main',
        ],
    },
)
