from setuptools import setup

package_name = 'control_hub'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='alithethird',
    maintainer_email='aliugur997@gmail.com',
    description='This is a Control Hub package to achieve communication between Test Robot and UI',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "control_hub = control_hub.control_hub:main"
            ],
    },
)
