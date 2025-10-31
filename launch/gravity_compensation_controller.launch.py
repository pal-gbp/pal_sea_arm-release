# Copyright (c) 2024 PAL Robotics S.L. All rights reserved.
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
import os
from ament_index_python.packages import get_package_share_directory
from controller_manager.launch_utils import generate_load_controller_launch_description
from launch.actions import GroupAction, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch import LaunchDescription, LaunchContext

from launch_pal.arg_utils import LaunchArgumentsBase, read_launch_argument
from launch_pal.param_utils import parse_parametric_yaml
from launch.actions import DeclareLaunchArgument, SetLaunchConfiguration
from dataclasses import dataclass
from launch_pal.robot_arguments import CommonArgs


@dataclass(frozen=True)
class LaunchArguments(LaunchArgumentsBase):
    side: DeclareLaunchArgument = CommonArgs.side

    mode: DeclareLaunchArgument = DeclareLaunchArgument(
        name='mode',
        default_value='effort',
        choices=['effort', 'torque'],
        description='Mode of the gravity compensation controller.')


def declare_actions(launch_description: LaunchDescription, launch_args: LaunchArguments):

    launch_description.add_action(OpaqueFunction(
        function=setup_controller_configuration))

    gravity_compensation_controller = GroupAction([generate_load_controller_launch_description(
        controller_name=LaunchConfiguration("controller_name"),
        controller_params_file=LaunchConfiguration("controller_config"),
        extra_spawner_args=["--inactive"])])

    launch_description.add_action(gravity_compensation_controller)

    return


def setup_controller_configuration(context: LaunchContext):

    side = read_launch_argument('side', context)
    mode = read_launch_argument('mode', context)

    arm_prefix = "arm"
    if side:
        arm_prefix = f"arm_{side}"

    controller_name = f"{arm_prefix}_gravity_compensation_controller"
    if mode == "torque":
        controller_name += "_" + mode

    remappings = {"ARM_SIDE_PREFIX": arm_prefix}

    param_file = os.path.join(
        get_package_share_directory('pal_sea_arm_controller_configuration'),
        'config', f'arm_gravity_compensation_controller_{mode}.yaml')

    parsed_yaml = parse_parametric_yaml(source_files=[param_file], param_rewrites=remappings)

    return [SetLaunchConfiguration('controller_name', controller_name),
            SetLaunchConfiguration('controller_config', parsed_yaml)]


def generate_launch_description():

    # Create the launch description
    ld = LaunchDescription()

    launch_arguments = LaunchArguments()

    launch_arguments.add_to_launch_description(ld)

    declare_actions(ld, launch_arguments)

    return ld
