from setuptools import setup

package_name = 'examples_rclpy_guard_conditions'

setup(
    name=package_name,
    version='0.18.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Aditya Pande, Alejandro Hernandez Cordero',
    maintainer_email='aditya.pande@openrobotics.org, alejandro@openrobotics.org',
    keywords=['ROS'],
    classifiers=[
        'Intended Audience :: Developers',
        'License :: OSI Approved :: Apache Software License',
        'Programming Language :: Python',
        'Topic :: Software Development',
    ],
    description='Examples of using guard conditions.',
    license='Apache License, Version 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'trigger_guard_condition = '
            'examples_rclpy_guard_conditions.trigger_guard_condition:main'
        ],
    },
)
