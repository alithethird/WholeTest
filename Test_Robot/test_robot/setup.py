from setuptools import setup

package_name = 'test_robot'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    author='Ali Ugur',
    author_email='aliugur997@gmail.com',
    maintainer='Ali Ugur',
    maintainer_email='aliugur997@gmail.com',
    keywords=['ROS'],
    classifiers=[
        'License :: OSI Approved :: Apache Software License',
        'Programming Language :: Python',
    ],
    description='ROS2 Action Server to control Raspberry Pi GPIO on Test Robot',
    license='Apache License, Version 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'test_robot_server = ' + package_name + '.test_robot_server:main',
        ],
    },
)
