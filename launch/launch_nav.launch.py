import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    package_name = "my_bot"

    # Map server
    pkg_path = os.path.join(get_package_share_directory(package_name))
    map_file = os.path.join(pkg_path, "config", "labyrinth.yaml")
    map_server = Node(
        package="nav2_map_server",
        executable="map_server",
        name="map_server",
        output="screen",
        parameters=[
            {"use_sim_time": True},
            {"yaml_filename": map_file},
        ],
    )

    # Rviz
    rviz_config_file = os.path.join(pkg_path, "config", "nav.rviz")
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        parameters=[
            {"use_sim_time": True},
            {"config": rviz_config_file},
        ],
        output="screen",
    )

    # Lifecycle manager
    lifecycle_manager = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager",
        output="screen",
        parameters=[
            {"use_sim_time": True},
            {"autostart": True},
            {"node_names": ["map_server"]},
        ],
    )

    # Robot state publisher
    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                os.path.join(
                    get_package_share_directory(package_name), "launch", "rsp.launch.py"
                )
            ]
        ),
        launch_arguments={"use_sim_time": "true"}.items(),
    )

    # Gazebo
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("gazebo_ros"), "launch", "gazebo.launch.py"
            ),
        ),
        launch_arguments=[
            ("world", os.path.join(pkg_path, "worlds", "labyrinth.world")),
            ("use_sim_time", "true"),
        ],
    )

    # Run the spawner node from the gazebo_ros package. The entity name doesn't really matter if you only have a single robot.
    spawn_entity = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=["-topic", "robot_description", "-entity", "my_bot"],
        output="screen",
    )

    # Static transform publisher
    static_transform_publisher = ExecuteProcess(
        cmd=[
            "ros2",
            "run",
            "tf2_ros",
            "static_transform_publisher",
            "0",
            "0",
            "0",
            "0",
            "0",
            "0",
            "map",
            "odom",
        ],
        output="screen",
    )

    # Run navigation_launch.py from nav2_bringup
    navigation_launch = ExecuteProcess(
        cmd=["ros2", "launch", "nav2_bringup", "navigation_launch.py"], output="screen"
    )

    return LaunchDescription(
        [
            map_server,
            # rviz,
            lifecycle_manager,
            rsp,
            gazebo,
            spawn_entity,
            static_transform_publisher,
            navigation_launch,
        ]
    )
