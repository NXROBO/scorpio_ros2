# Copyright (c) 2025 NXROBO
#
# /* Author: litian.zhuang */
# /* email: litian.zhuang@nxrobo.com */
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
# 

import os
import launch
import launch_ros
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share_dir = get_package_share_directory('scorpio_base')

    # arg_serial_motor_port = DeclareLaunchArgument('serial_motor_port', default_value='/dev/ttyS3',
    #                                      description='serial motor port name:/dev/ttyS3 or /dev/ttyUSBx')
    arg_serial_stm32_port = DeclareLaunchArgument('serial_stm32_port', default_value='/dev/ttyS0',
                                         description='serial motor port name:/dev/ttyS0 or /dev/ttyUSBx')
    arg_base_frame_id = DeclareLaunchArgument('base_frame_id', default_value='base_footprint',
                                           description='base fram id')
    arg_odom_frame_id = DeclareLaunchArgument('odom_frame_id', default_value='odom',
                                                description='Base link frame id')
    arg_frame_id = DeclareLaunchArgument('frame_id', default_value='odom',
                                                description='frame id')
    arg_wheelbase = DeclareLaunchArgument('wheelbase', default_value=['0.315'],
                                                description='wheelbase')    

    arg_params_file = DeclareLaunchArgument('params_file',
                                           default_value=os.path.join(
                                           share_dir, 'cfg', 'device_config.yaml'),
                                           description='FPath to the ROS2 parameters file to use.')

    scorpio_base_node = launch_ros.actions.Node(
        package='scorpio_base',
        executable='scorpio_base_node',  
        output='screen',
        emulate_tty=True,
        parameters=[{
                # 'serial_motor_port': launch.substitutions.LaunchConfiguration('serial_motor_port'),           
                'arg_serial_stm32_port': launch.substitutions.LaunchConfiguration('serial_stm32_port'),                
                'base_frame_id': launch.substitutions.LaunchConfiguration('base_frame_id'),
                'odom_frame_id': launch.substitutions.LaunchConfiguration('odom_frame_id'),
                
        }, '/opt/nxrobo/device_config.yaml' ])
    
    scorpio_ackermann_node = launch_ros.actions.Node(
        package='scorpio_base',
        executable='cmd_vel_to_ackermann_cmd_node',  
        output='screen',
        emulate_tty=True,
        parameters=[{
                'frame_id': launch.substitutions.LaunchConfiguration('frame_id'),
                'wheelbase': launch.substitutions.LaunchConfiguration('wheelbase')
        }])
    return LaunchDescription([
        # arg_serial_motor_port,     
        arg_serial_stm32_port,   
        arg_base_frame_id,
        arg_odom_frame_id,
        arg_wheelbase,
        arg_frame_id,
        arg_params_file,
        scorpio_base_node,
        scorpio_ackermann_node
    ])
