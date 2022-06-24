# Copyright 2021 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the 'License');
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an 'AS IS' BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Launch the GPU pipeline for Julia Set Example."""

import platform

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

RESOLUTIONS = {'16K': (15360, 8640),
               '8K': (7680, 4320),
               '4K': (3840, 2160),
               '1080p': (1920, 1080),
               '720p': (1280, 720),
               '480p': (852, 480)}
IMAGE_HZ = 100.0

MAX_ITERATION = 50

JULIASET_PARAMS = [{'min_x_range': -2.5},
                   {'max_x_range': 2.5},
                   {'min_y_range': -1.5},
                   {'max_y_range': 1.5},
                   {'start_x': 0.7885},
                   {'start_y': 0.7885},
                   {'boundary_radius': 16.0},
                   {'max_iterations': MAX_ITERATION}]

launch_args = [DeclareLaunchArgument('enable_type_adapt', default_value='true',
                                     description='Enable type adaptation mode'),
               DeclareLaunchArgument('resolution', default_value='1080p',
                                     description='Resolution key (16K|8K|4K|1080p|720p|480p)'),
               DeclareLaunchArgument('enable_mt', default_value='false',
                                     description='Enable multithreaded composable containers'),
               DeclareLaunchArgument('enable_nsys', default_value='false',
                                     description='Enable nsys profiling'),
               DeclareLaunchArgument('nsys_profile_label', default_value='',
                                     description='Label to append for nsys profile output'),
               DeclareLaunchArgument('nsys_profile_flags', default_value='--trace=osrt,nvtx,cuda',
                                     description='Flags for nsys profile')
               ]


def generate_launch_description():
    """Generate launch description with cam2image feeding N JuliaSetNode pipeline."""
    ld = LaunchDescription(launch_args)
    ld.add_action(OpaqueFunction(function=launch_setup))
    return ld


def launch_setup(context):
    enable_type_adapt = IfCondition(LaunchConfiguration('enable_type_adapt')).evaluate(context)
    resolution = LaunchConfiguration('resolution').perform(context)
    enable_mt = IfCondition(LaunchConfiguration('enable_mt')).evaluate(context)
    enable_nsys = IfCondition(LaunchConfiguration('enable_nsys')).evaluate(context)
    nsys_profile_label = LaunchConfiguration('nsys_profile_label').perform(context)
    nsys_profile_flags = LaunchConfiguration('nsys_profile_flags').perform(context)

    container_prefix = ''

    if enable_nsys:
        nsys_profile_name = build_profile_name(
            nsys_profile_label, enable_type_adapt, enable_mt, resolution)
        container_prefix = f'nsys profile {nsys_profile_flags} -o {nsys_profile_name}'

    cam2image_node = ComposableNode(package='image_tools',
                                    name='cam2image',
                                    plugin='image_tools::Cam2Image',
                                    remappings=[('/image', '/image_in')],
                                    extra_arguments=[
                                        {'use_intra_process_comms': True}],
                                    parameters=[{'burger_mode': True,
                                                 'history': 'keep_last',
                                                 'frequency': IMAGE_HZ,
                                                 'width': RESOLUTIONS[resolution][0],
                                                 'height': RESOLUTIONS[resolution][1]}])

    pipeline_nodes = [cam2image_node]

    pipeline_nodes.append(ComposableNode(
        package='julia_set_example',
        plugin='type_adapt_example::MapNode',
        name='map_node',
        parameters=[{'type_adaptation_enabled': enable_type_adapt}] + JULIASET_PARAMS,
        remappings=[('/image_out', '/image_out0')]))

    for i in range(1, MAX_ITERATION):
        pipeline_nodes.append(ComposableNode(
            package='julia_set_example',
            plugin='type_adapt_example::JuliaSetNode',
            name='juliaset_node%d' % (i),
            parameters=[{'type_adaptation_enabled': enable_type_adapt},
                        {'proc_id': i}] + JULIASET_PARAMS,
            remappings=[('/image_in', '/image_out%d' % (i - 1)),
                        ('/image_out', '/image_out%d' % (i))]))

    pipeline_nodes.append(ComposableNode(
        package='julia_set_example',
        plugin='type_adapt_example::ColorizeNode',
        name='colorize_node',
        parameters=[{'max_iterations': MAX_ITERATION},
                    {'type_adaptation_enabled': enable_type_adapt}] + JULIASET_PARAMS,
        remappings=[('/image_in', '/image_out%d' % (MAX_ITERATION - 1)),
                    ('/image_out', '/pipeline/image_out')]))

    pipeline_container = ComposableNodeContainer(
        name='pipeline_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container' + ('_mt' if enable_mt else ''),
        prefix=container_prefix,
        sigkill_timeout='50' if enable_nsys else '5',
        sigterm_timeout='50' if enable_nsys else '5',
        composable_node_descriptions=pipeline_nodes,
        output='both'
    )

    return [pipeline_container]


def build_profile_name(label, enable_type_adapt, enable_mt, resolution):
    return f"ros-type_adapt-{platform.machine()}{'' if not enable_mt else '-mt'}" +\
        f"-{enable_type_adapt}-{resolution}-{int(IMAGE_HZ)}hz{'' if not label else '-' + label}"
