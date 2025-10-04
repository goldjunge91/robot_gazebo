# Copyright 2024 Husarion sp. z o.o.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    EnvironmentVariable,
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch_ros.actions import Node, PushROSNamespace, SetParameter, SetRemap
from launch_ros.substitutions import FindPackageShare
from nav2_common.launch import ReplaceString


def generate_launch_description():
    components_config = LaunchConfiguration("components_config")
    configuration = LaunchConfiguration("configuration")
    namespace = LaunchConfiguration("namespace")
    robot_model = LaunchConfiguration("robot_model")
    x = LaunchConfiguration("x")
    y = LaunchConfiguration("y")
    z = LaunchConfiguration("z")
    roll = LaunchConfiguration("roll")
    pitch = LaunchConfiguration("pitch")
    yaw = LaunchConfiguration("yaw")

    components_file = PythonExpression(["'", configuration, "' + '.yaml'"])
    default_components_config = PathJoinSubstitution(
        [FindPackageShare("robot_description"), "config", robot_model, components_file]
    )
    declare_components_config_arg = DeclareLaunchArgument(
        "components_config",
        default_value=default_components_config,
        description=(
            "Specify file which contains components. These components will be included in URDF. "
            "Available options can be found in [husarion_components_description](https://github.com/husarion/husarion_components_description/blob/ros2/README.md#available-urdf-sensors)"
        ),
    )

    declare_configuration_arg = DeclareLaunchArgument(
        "configuration",
        default_value="basic",
        description=(
            "Specify configuration packages. Currently only ROSbot XL has available packages"
        ),
        choices=["basic", "telepresence", "autonomy", "manipulation", "manipulation_pro"],
    )

    declare_namespace_arg = DeclareLaunchArgument(
        "namespace",
        default_value=EnvironmentVariable("ROBOT_NAMESPACE", default_value=""),
        description="Add namespace to all launched nodes.",
    )

    declare_robot_model_arg = DeclareLaunchArgument(
        "robot_model",
        default_value=EnvironmentVariable("ROBOT_MODEL_NAME", default_value="robot_xl"),
        description="Specify robot model",
        choices=["robot", "robot_xl"],
    )

    declare_x_arg = DeclareLaunchArgument(
        "x", default_value="0.0", description="Initial robot position in the global 'x' axis."
    )

    declare_y_arg = DeclareLaunchArgument(
        "y", default_value="0.0", description="Initial robot position in the global 'y' axis."
    )

    declare_z_arg = DeclareLaunchArgument(
        "z", default_value="0.0", description="Initial robot position in the global 'z' axis."
    )

    declare_roll_arg = DeclareLaunchArgument(
        "roll", default_value="0.0", description="Initial robot 'roll' orientation."
    )

    declare_pitch_arg = DeclareLaunchArgument(
        "pitch", default_value="0.0", description="Initial robot 'pitch' orientation."
    )

    declare_yaw_arg = DeclareLaunchArgument(
        "yaw", default_value="0.0", description="Initial robot 'yaw' orientation."
    )

    ns = PythonExpression(["'", namespace, "' + '/' if '", namespace, "' else ''"])
    robot_name = PythonExpression(
        ["'robot'", " if '", namespace, "' == '' ", "else ", "'", namespace, "'"]
    )

    gz_spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-name",
            robot_name,
            "-allow_renaming",
            "true",
            "-topic",
            "robot_description",
            "-x",
            x,
            "-y",
            y,
            "-z",
            z,
            "-R",
            roll,
            "-P",
            pitch,
            "-Y",
            yaw,
        ],
    )

    welcome_msg = LogInfo(
        msg=[
            "Spawning ROSbot\n\tNamespace: '",
            namespace,
            "'\n\tInitial pose: (",
            x,
            ", ",
            y,
            ", ",
            z,
            ", ",
            roll,
            ", ",
            pitch,
            ", ",
            yaw,
            ")",
        ]
    )

    gz_bridge_path = PathJoinSubstitution(
        [FindPackageShare("robot_gazebo"), "config", "robot_bridge.yaml"]
    )
    gz_bridge_config = ReplaceString(gz_bridge_path, {"<namespace>/": ns})

    gz_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="robot_gz_bridge",
        parameters=[{"config_file": gz_bridge_config}],
    )

    controller_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("robot_controller"),
                    "launch",
                    "controller.launch.py",
                ]
            )
        ),
        launch_arguments={
            "components_config": components_config,
            "configuration": configuration,
            "robot_model": robot_model,
            "use_sim": "True",
        }.items(),
    )

    husarion_components_description = FindPackageShare("husarion_components_description")
    robot_localization = FindPackageShare("robot_localization")
    robot_utils = FindPackageShare("robot_utils")

    gz_components = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [husarion_components_description, "launch", "gz_components.launch.py"]
            )
        ),
        launch_arguments={
            "components_config_path": components_config,
        }.items(),
    )

    localization_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([robot_localization, "launch", "ekf.launch.py"])
        ),
    )

    laser_filter_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([robot_utils, "launch", "laser_filter.launch.py"])
        ),
        launch_arguments={"robot_model": robot_model}.items(),
    )

    return LaunchDescription(
        [
            declare_configuration_arg,
            declare_namespace_arg,
            declare_robot_model_arg,
            declare_components_config_arg,  # depends on configuration and robot model
            declare_x_arg,
            declare_y_arg,
            declare_z_arg,
            declare_roll_arg,
            declare_pitch_arg,
            declare_yaw_arg,
            PushROSNamespace(namespace),
            SetRemap("/diagnostics", "diagnostics"),
            SetRemap("/tf", "tf"),
            SetRemap("/tf_static", "tf_static"),
            SetParameter(name="use_sim_time", value=True),
            welcome_msg,
            gz_bridge,
            gz_spawn_entity,
            gz_components,
            controller_launch,
            localization_launch,
            laser_filter_launch,
        ]
    )
