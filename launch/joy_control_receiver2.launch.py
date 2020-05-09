#!/usr/bin/env python3
#
# =======================================================================
#   @file   joy_control_receiver2.launch.py
#   @brief
#   @note
#
#   Copyright (C) 2020 Yasushi Oshima (oosmyss@gmail.com)
# =======================================================================

import launch
import launch.actions
import launch.substitutions
import launch_ros.actions


def generate_launch_description():
    joy_params_file = launch.substitutions.LaunchConfiguration(
        'joy_params',
        default=[launch.substitutions.ThisLaunchFileDir(), '/joy.yaml']
    )

    return launch.LaunchDescription([
        launch_ros.actions.Node(
            package='joy', node_executable='joy_node', output='screen',
            parameters=[joy_params_file]),
        launch_ros.actions.Node(
            package='joy_control_receiver2', node_executable='joy_control_receiver', output='screen'),
    ])
