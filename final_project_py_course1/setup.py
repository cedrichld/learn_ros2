from setuptools import find_packages, setup

package_name = 'final_project_py_course1'

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
            "turtle_spawn = final_project_py_course1.turtle_spawn:main",
            "turtle_kill = final_project_py_course1.turtle_kill:main",
            "pid_turtle = final_project_py_course1.pid_turtle:main",
            "find_target = final_project_py_course1.find_target:main",
        ],
    },
)
