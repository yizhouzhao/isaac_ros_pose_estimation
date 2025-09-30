# SPDX-FileCopyrightText: NVIDIA CORPORATION & AFFILIATES
# Copyright (c) 2024 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# SPDX-License-Identifier: Apache-2.0

from typing import Any, Dict, List

from isaac_ros_examples import IsaacROSLaunchFragment
import launch
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

# Expected number of input messages in 1 second
INPUT_IMAGES_EXPECT_FREQ = 30
# Number of input messages to be dropped in 1 second
INPUT_IMAGES_DROP_FREQ = 28

# RT-DETR models expect 640x640 encoded image size
RT_DETR_MODEL_INPUT_SIZE = 640
# RT-DETR models expect 3 image channels
RT_DETR_MODEL_NUM_CHANNELS = 3

VISUALIZATION_DOWNSCALING_FACTOR = 10

REFINE_ENGINE_PATH = '/tmp/refine_trt_engine.plan'
SCORE_ENGINE_PATH = '/tmp/score_trt_engine.plan'


def generate_launch_description():
    launch_args = [
            DeclareLaunchArgument(
                'mesh_file_path',
                default_value='',
                description='The absolute file path to the mesh file'),

            DeclareLaunchArgument(
                'texture_path',
                default_value='',
                description='The absolute file path to the texture map'),

            DeclareLaunchArgument(
                'refine_engine_file_path',
                default_value=REFINE_ENGINE_PATH,
                description='The absolute file path to the refine trt engine'),

            DeclareLaunchArgument(
                'score_engine_file_path',
                default_value=SCORE_ENGINE_PATH,
                description='The absolute file path to the score trt engine')
        ]
    
    # FoundationPose parameters
    mesh_file_path = LaunchConfiguration('mesh_file_path')
    texture_path = LaunchConfiguration('texture_path')
    refine_engine_file_path = LaunchConfiguration('refine_engine_file_path')
    score_engine_file_path = LaunchConfiguration('score_engine_file_path')

    # FoundationPose pipeline
    foundationpose_node = ComposableNode(
            name='foundationpose_node',
            package='isaac_ros_foundationpose',
            plugin='nvidia::isaac_ros::foundationpose::FoundationPoseNode',
            parameters=[{
                'mesh_file_path': mesh_file_path,
                'texture_path': texture_path,

                'refine_engine_file_path': refine_engine_file_path,
                'refine_input_tensor_names': ['input_tensor1', 'input_tensor2'],
                'refine_input_binding_names': ['input1', 'input2'],
                'refine_output_tensor_names': ['output_tensor1', 'output_tensor2'],
                'refine_output_binding_names': ['output1', 'output2'],

                'score_engine_file_path': score_engine_file_path,
                'score_input_tensor_names': ['input_tensor1', 'input_tensor2'],
                'score_input_binding_names': ['input1', 'input2'],
                'score_output_tensor_names': ['output_tensor'],
                'score_output_binding_names': ['output1'],
            }],
            remappings=[
                ('pose_estimation/depth_image', 'depth_image2'),
                ('pose_estimation/image', 'rgb/image_rect_color2'),
                ('pose_estimation/camera_info', 'rgb/camera_info2'),
                ('pose_estimation/segmentation', 'segmentation2'),
                ('pose_estimation/output', 'output2')]
        )
    

    input_width = 640
    input_height = 480
    input_to_RT_DETR_ratio = input_width / RT_DETR_MODEL_INPUT_SIZE
    
    # Create a binary segmentation mask from a Detection2DArray published by RT-DETR.
    # The segmentation mask is of size
    # int(IMAGE_WIDTH/input_to_RT_DETR_ratio) x int(IMAGE_HEIGHT/input_to_RT_DETR_ratio)
    # detection2_d_array_filter = ComposableNode(
    #         name='detection2_d_array_filter',
    #         package='isaac_ros_foundationpose',
    #         plugin='nvidia::isaac_ros::foundationpose::Detection2DArrayFilter',
    #         remappings=[('detection2_d_array', 'detections_output')]
    #     )
    
    # detection2_d_to_mask = ComposableNode(
    #         name='detection2_d_to_mask',
    #         package='isaac_ros_foundationpose',
    #         plugin='nvidia::isaac_ros::foundationpose::Detection2DToMask',
    #         parameters=[{
    #             'mask_width': int(input_width/input_to_RT_DETR_ratio),
    #             'mask_height': int(input_height/input_to_RT_DETR_ratio)
    #         }],
    #         remappings=[('segmentation', 'rt_detr_segmentation')]
    #     )


    foundationpose_container = ComposableNodeContainer(
        package='rclcpp_components',
        name='foundationpose_container',
        namespace='',
        executable='component_container_mt',
        composable_node_descriptions=[foundationpose_node],
        # composable_node_descriptions=[foundationpose_node, detection2_d_array_filter, detection2_d_to_mask],
        output='screen'
    )

    return launch.LaunchDescription(
        launch_args + [foundationpose_container])