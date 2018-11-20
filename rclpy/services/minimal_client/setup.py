from setuptools import setup

package_name = 'examples_rclpy_minimal_client'

setup(
    name=package_name,
    version='0.6.0',
    packages=[],
    py_modules=[
        'client',
        'client_async',
        'client_async_member_function',
        'client_async_callback'],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    author='Mikael Arguedas',
    author_email='mikael@osrfoundation.org',
    maintainer='Mikael Arguedas',
    maintainer_email='mikael@osrfoundation.org',
    keywords=['ROS'],
    classifiers=[
        'Intended Audience :: Developers',
        'License :: OSI Approved :: Apache Software License',
        'Programming Language :: Python',
        'Topic :: Software Development',
    ],
    description='Examples of minimal service clients using rclpy.',
    license='Apache License, Version 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'client = client:main',
            'client_async = client_async:main',
            'client_async_member_function ='
            ' client_async_member_function:main',
            'client_async_callback = client_async_callback:main',
        ],
    },
)
