from setuptools import setup

package_name = 'examples_rclpy_actions'

setup(
    name=package_name,
    version='0.1.0',
    packages=[],
    py_modules=[
        'client',
        'server',
        'server_parallel_goals'],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    author='Shane Loretz',
    author_email='sloretz@openrobotics.org',
    maintainer='Shane Loretz',
    maintainer_email='sloretz@openrobotics.org',
    keywords=['ROS'],
    classifiers=[
        'Intended Audience :: Developers',
        'License :: OSI Approved :: Apache Software License',
        'Programming Language :: Python',
        'Topic :: Software Development',
    ],
    description='Examples of action services and clients using rclpy.',
    license='Apache License, Version 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'client = server:main',
            'server = server:main',
            'server_parallel_goals = server_parallel_goals:main',
        ],
    },
)
