from setuptools import setup

package_name = 'launch_testing_examples'

setup(
    name=package_name,
    version='0.20.3',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Aditya Pande, Alejandro Hernandez Cordero',
    maintainer_email='aditya.pande@openrobotics.org, alejandro@openrobotics.org',
    description='Examples of simple launch tests',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
