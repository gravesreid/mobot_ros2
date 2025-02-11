from setuptools import setup

package_name = 'mobot_ros2'

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
    maintainer='reid',
    maintainer_email='gravesreid@gmail.com',
    description='ros2 package for the mobot robot',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'joystick_controller = mobot_ros2.joystick_controller:main'
        ],
    },
)
