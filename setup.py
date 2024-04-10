from setuptools import find_packages, setup

package_name = 'srv_call_timer'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Priyanshu Agrawal',
    maintainer_email='priyanshu.agrawal@uconn.edu',
    description='A tool to measure the time it takes to call a ROS2 service and receive the result.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'srv_call_timer = srv_call_timer.srv_call_timer:main'
        ],
    },
)
