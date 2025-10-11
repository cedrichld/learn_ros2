from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    ld = LaunchDescription()

    param_config = os.path.join(get_package_share_directory("course_level1_py"),
                                "config", "number_app.yaml")

    number_publisher = Node(
        package="course_level1_py",
        executable="number_publisher",
        namespace="/abc",
        name="my_nb_pub",
        remappings=[("number", "my_nb")],
        # parameters=[
        #     {"number": 12},
        #     {"timer_period": 1.3}
        # ]
        parameters=[param_config]
    )

    number_counter = Node(
        package="course_level1_cpp",
        executable="number_publisher" ,
        namespace="/abc",
        name="my_nb_count",
        remappings=[("number", "my_nb")]
    )

    ld.add_action(number_publisher)
    ld.add_action(number_counter)

    return ld