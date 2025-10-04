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
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, SetParameter, SetRemap
from launch_ros.substitutions import FindPackageShare
import logging
from launch.actions import OpaqueFunction


def generate_launch_description():
    rviz = LaunchConfiguration("rviz")
    include_nerf_launcher = LaunchConfiguration("include_nerf_launcher")
    declare_rviz_arg = DeclareLaunchArgument(
        "rviz",
        default_value="True",
        description="Run RViz simultaneously.",
        choices=["True", "true", "False", "false"],
    )

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("husarion_gz_worlds"), "launch", "gz_sim.launch.py"]
            )
        ),
        launch_arguments={"gz_log_level": "1"}.items(),
    )

    def _maybe_include_gz_sim(context, *args, **kwargs):
        # Try resolving the package share; if not available, skip including gz_sim
        try:
            # This will raise if the package is not found at runtime
            _ = FindPackageShare("husarion_gz_worlds").perform(context)
            return [gz_sim]
        except Exception as e:
            logging.getLogger("robot_gazebo.launch").warning(
                "Package 'husarion_gz_worlds' not found, skipping gz_sim include: %s", e
            )
            return []

    gz_sim_wrapper = OpaqueFunction(function=_maybe_include_gz_sim)

    gz_bridge_config = PathJoinSubstitution(
        [FindPackageShare("robot_gazebo"), "config", "gz_bridge.yaml"]
    )
    gz_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="gz_bridge",
        parameters=[{"config_file": gz_bridge_config}],
    )

    spawn_robot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("robot_gazebo"),
                    "launch",
                    "spawn_robot.launch.py",
                ]
            )
        ),
        launch_arguments={"include_nerf_launcher": include_nerf_launcher}.items(),
    )

    rviz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("robot_description"),
                    "launch",
                    "rviz.launch.py",
                ]
            )
        ),
        launch_arguments={"namespace": ""}.items(),
        condition=IfCondition(rviz),
    )

    declare_include_nerf_arg = DeclareLaunchArgument(
        "include_nerf_launcher",
        default_value="False",
        description="Include Nerf launcher in the robot URDF",
        choices=["True", "False"],
    )

    return LaunchDescription(
        [
            declare_rviz_arg,
            declare_include_nerf_arg,
            SetRemap("/diagnostics", "diagnostics"),
            SetRemap("/tf", "tf"),
            SetRemap("/tf_static", "tf_static"),
            SetParameter(name="use_sim_time", value=True),
            gz_sim_wrapper,
            gz_bridge,
            spawn_robot,
            rviz_launch,
        ]
    )
