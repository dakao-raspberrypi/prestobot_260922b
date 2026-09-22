from setuptools import find_packages, setup

package_name = 'prestobot_py_pkg'

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
    maintainer='dakao',
    maintainer_email='dakao@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "set_initial_pose = prestobot_py_pkg.set_initial_pose:main",
            "hmi = prestobot_py_pkg.hmi:main",
            "topic_relay = prestobot_py_pkg.topic_relay:main",
            "mpu6050_driver = prestobot_py_pkg.mpu6050_driver:main",
            "simple_serial_receiver = prestobot_py_pkg.simple_serial_receiver:main",
            "simple_serial_transmitter = prestobot_py_pkg.simple_serial_transmitter:main",
            "nav_error_monitor = prestobot_py_pkg.nav_error_monitor:main"
        ],
    },
)
