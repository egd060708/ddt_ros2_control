# Copyright 2025
# SPDX-License-Identifier: Apache-2.0

import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _workspace_data_directory() -> str:
    try:
        share = Path(get_package_share_directory('interaction_data_recorder')).resolve()
        for parent in share.parents:
            if (parent / 'src').is_dir():
                return str(parent / 'data')
    except Exception:
        pass
    return str(Path.cwd() / 'data')


def _launch_setup(context, *_args, **_kwargs):
    """params_file 先加载；launch 仅在 CLI 显式给出非空值时才覆盖对应项（否则会盖掉 yaml）。"""
    out_csv = LaunchConfiguration('output_csv_filename').perform(context).strip()
    sample_hz = LaunchConfiguration('sample_rate_hz').perform(context).strip()

    overrides = {
        'use_sim_time': ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool),
        'sequence_file': LaunchConfiguration('sequence_file'),
        'output_directory': LaunchConfiguration('output_directory'),
    }
    # 仅当用户在命令行传入非空时才覆盖（默认留空则沿用 recorder_params.yaml）
    if out_csv:
        overrides['output_csv_filename'] = out_csv
    if sample_hz:
        overrides['sample_rate_hz'] = sample_hz

    return [
        Node(
            package='interaction_data_recorder',
            executable='recorder_node',
            output='screen',
            parameters=[LaunchConfiguration('params_file'), overrides],
        )
    ]


def generate_launch_description() -> LaunchDescription:
    pkg = get_package_share_directory('interaction_data_recorder')
    default_seq = os.path.join(pkg, 'config', 'sequence_example.yaml')
    default_params = os.path.join(pkg, 'config', 'recorder_params.yaml')
    default_out = _workspace_data_directory()

    return LaunchDescription(
        [
            DeclareLaunchArgument('use_sim_time', default_value='true'),
            DeclareLaunchArgument('sequence_file', default_value=default_seq),
            DeclareLaunchArgument('params_file', default_value=default_params),
            DeclareLaunchArgument(
                'output_directory',
                default_value=default_out,
                description='CSV 输出目录，默认同节点：<工作空间>/data',
            ),
            DeclareLaunchArgument(
                'output_csv_filename',
                default_value='',
                description='留空则使用 params_file 中的值；仅在命令行传入非空时覆盖 yaml',
            ),
            DeclareLaunchArgument(
                'sample_rate_hz',
                default_value='',
                description='留空则使用 params_file；传入数字则覆盖（如 200.0）',
            ),
            OpaqueFunction(function=_launch_setup),
        ]
    )
