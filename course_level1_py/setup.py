from setuptools import find_packages, setup

package_name = 'course_level1_py'

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
    maintainer='ros2',
    maintainer_email='ros2@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "robot_news_station = course_level1_py.robot_news_station:main",
            "smartphone = course_level1_py.smartphone:main",
            "number_publisher = course_level1_py.number_publisher:main",
            "number_counter = course_level1_py.number_counter:main",
            "add_two_ints_server = course_level1_py.add_two_ints_server:main",
            "add_two_ints_client_noop = course_level1_py.add_two_ints_client_noop:main",
            "add_two_ints_client = course_level1_py.add_two_ints_client:main",
            "number_reset_counter_client = course_level1_py.number_reset_counter_client:main",
            "hardware_status_publisher = course_level1_py.hardware_status_publisher:main",
            "battery_client = course_level1_py.battery_node:main",
            "led_panel = course_level1_py.led_panel_node:main",
        ],
    },
)
