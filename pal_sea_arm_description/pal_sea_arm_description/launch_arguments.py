# Copyright (c) 2023 PAL Robotics S.L. All rights reserved.
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


from dataclasses import dataclass
from launch.actions import DeclareLaunchArgument as DLA
from launch_pal.arg_utils import parse_launch_args_from_yaml
from ament_index_python.packages import get_package_share_directory


@dataclass(frozen=True)
class SEAArmArgs:
    """This dataclass contains launch arguments for the pal_sea_arm."""

    __robot_name = 'pal_sea_arm'
    __pkg_dir = get_package_share_directory(f"{__robot_name}_description")
    __arg_creator = parse_launch_args_from_yaml(
        f"{__pkg_dir}/config/{__robot_name}_configuration.yaml")

    arm_type: DLA = __arg_creator.get_argument("arm_type")
    end_effector: DLA = __arg_creator.get_argument("end_effector")
    ft_sensor: DLA = __arg_creator.get_argument("ft_sensor")
    wrist_model: DLA = __arg_creator.get_argument("wrist_model")
    tool_changer: DLA = __arg_creator.get_argument("tool_changer")
    torque_estimation: DLA = __arg_creator.get_argument("torque_estimation")
