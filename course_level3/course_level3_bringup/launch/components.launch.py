from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

def generate_launch_description():
    ld = LaunchDescription()

    container = ComposableNodeContainer(
        name="my_container",
        namespace="",
        package="rclcpp_components",
        executable="component_container",
        composable_node_descriptions= [
            ComposableNode(
                package="components_cpp",
                plugin="my_namespace::NumberPublisher",
                name="number_publisher_1"
            ),
            ComposableNode(
                package="components_cpp",
                plugin="my_namespace::NumberPublisher",
                name="number_publisher_2"
            )
        ]
    )
    ld.add_action(container)

    return ld