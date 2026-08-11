import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import OpaqueFunction
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode

def launch_setup(context, *args, **kwargs):
    pkg_dir = get_package_share_directory('stereo_depth_yolo')
    
    width = 1280
    height = 720
    net_height = 736
    engine_file = 'model.plan'
    pad_y = 8.0

    left_yaml = os.path.join(pkg_dir, 'config', 'left.yaml')
    right_yaml = os.path.join(pkg_dir, 'config', 'right.yaml')

    # 1. Left Argus Mono Camera
    left_cam = ComposableNode(
        package='isaac_ros_argus_camera',
        plugin='nvidia::isaac_ros::argus::ArgusMonoNode',
        name='left_cam',
        namespace='left',
        parameters=[{
            'camera_id': 0,
            'camera_info_url': f'file://{left_yaml}',
            'output_encoding': 'mono8',
            'optical_frame_name': 'left_cam_optical'
        }],
        remappings=[
            ('left/image_raw', '/left/image_raw'),
            ('left/camera_info', '/left/camera_info')
        ]
    )

    # 2. Right Argus Mono Camera
    right_cam = ComposableNode(
        package='isaac_ros_argus_camera',
        plugin='nvidia::isaac_ros::argus::ArgusMonoNode',
        name='right_cam',
        namespace='right',
        parameters=[{
            'camera_id': 1,
            'camera_info_url': f'file://{right_yaml}',
            'output_encoding': 'mono8',
            'optical_frame_name': 'right_cam_optical'
        }],
        remappings=[
            ('left/image_raw', '/right/image_raw'),
            ('left/camera_info', '/right/camera_info')
        ]
    )

    # 3. C++ Zero-Copy Header Timestamp Sync Node (Component)
    header_sync_node = ComposableNode(
        package='stereo_depth_yolo',
        plugin='stereo_depth_yolo::StereoHeaderSyncNode',
        name='stereo_header_sync_node',
        extra_arguments=[{'use_intra_process_comms': True}]
    )

    # 4. Left Rectify Node (Consumes /synced/ left stream)
    left_rectify_node = ComposableNode(
        name='left_rectify_node',
        package='isaac_ros_image_proc',
        plugin='nvidia::isaac_ros::image_proc::RectifyNode',
        parameters=[{'output_width': width, 'output_height': height}],
        remappings=[
            ('image_raw', '/synced/left/image_raw'),
            ('camera_info', '/synced/left/camera_info'),
            ('image_rect', '/left/image_rect'),
            ('camera_info_rect', '/left/camera_info_rect')
        ]
    )

    # 5. Right Rectify Node (Consumes /synced/ right stream)
    right_rectify_node = ComposableNode(
        name='right_rectify_node',
        package='isaac_ros_image_proc',
        plugin='nvidia::isaac_ros::image_proc::RectifyNode',
        parameters=[{'output_width': width, 'output_height': height}],
        remappings=[
            ('image_raw', '/synced/right/image_raw'),
            ('camera_info', '/synced/right/camera_info'),
            ('image_rect', '/right/image_rect'),
            ('camera_info_rect', '/right/camera_info_rect')
        ]
    )

    # 6. Stereo Disparity Node (Clean CUDA Nitros execution)
    disparity_node = ComposableNode(
        name='disparity_node',
        package='isaac_ros_stereo_image_proc',
        plugin='nvidia::isaac_ros::stereo_image_proc::DisparityNode',
        parameters=[{
            'max_disparity': 128.0,
            'backend': 'CUDA'
        }],
        remappings=[
            ('left/image_rect', '/left/image_rect'),
            ('left/camera_info', '/left/camera_info_rect'),
            ('right/image_rect', '/right/image_rect'),
            ('right/camera_info', '/right/camera_info_rect'),
            ('disparity', '/disparity')
        ]
    )

    # 7. YOLO Image Encoder
    yolo_encoder = ComposableNode(
        name='yolo_encoder',
        package='isaac_ros_dnn_image_encoder',
        plugin='nvidia::isaac_ros::dnn_inference::DnnImageEncoderNode',
        parameters=[{
            'input_image_width': width,
            'input_image_height': height,
            'network_image_width': width,
            'network_image_height': net_height,
            'image_mean': [0.0, 0.0, 0.0],
            'image_stddev': [1.0, 1.0, 1.0],
        }],
        remappings=[
            ('image', '/left/image_rect'),
            ('camera_info', '/left/camera_info_rect'),
            ('encoded_tensor', '/tensor_pub')
        ]
    )

    # 8. TensorRT Node
    engine_path = os.path.join(pkg_dir, 'config', engine_file)
    tensorrt_node = ComposableNode(
        name='tensorrt_node',
        package='isaac_ros_tensor_rt',
        plugin='nvidia::isaac_ros::dnn_inference::TensorRTNode',
        parameters=[{
            'model_file_path': '',
            'engine_file_path': engine_path,
            'force_engine_update': False,
            'input_tensor_names': ['input_tensor'],
            'input_binding_names': ['images'],      
            'output_tensor_names': ['output_tensor'], 
            'output_binding_names': ['output0'],
            'input_dimensions': [1, 3, net_height, width]
        }],
        remappings=[
            ('tensor_sub', '/tensor_pub'),
            ('tensor_pub', '/tensor_sub')
        ]
    )

    # 9. YOLO Decoder Node
    yolo_decoder_node = ComposableNode(
        name='yolo_decoder_node',
        package='isaac_ros_yolov8',
        plugin='nvidia::isaac_ros::yolov8::YoloV8DecoderNode',
        parameters=[{
            'tensor_name': 'output_tensor',
            'confidence_threshold': 0.5,
            'nms_threshold': 0.45,
            'num_classes': 1 
        }],
        remappings=[
            ('tensor_sub', '/tensor_sub'),
            ('detections_output', '/detections_output')
        ]
    )

    # Single Container holding all GPU & C++ intra-process components
    container = ComposableNodeContainer(
        name='stereo_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container_mt',
        composable_node_descriptions=[
            left_cam,
            right_cam,
            #header_sync_node,
            left_rectify_node,
            right_rectify_node,
            disparity_node,
            yolo_encoder,
            tensorrt_node,
            yolo_decoder_node
        ],
        arguments=['--ros-args', '--log-level', 'info'],
        parameters=[{'thread_num': 8}], # Explicitly allocate threads
        output='screen'
    
    )

    # Python Spatial 3D Node (Reads output bounding boxes and depth map)
    spatial_detection_node = Node(
        package='stereo_depth_yolo',
        executable='spatial_detection_node.py',
        name='spatial_detection_node',
        parameters=[{'pad_y': pad_y}],
        output='screen'
    )

    return [
        container,
        spatial_detection_node
    ]

def generate_launch_description():
    return LaunchDescription([
        OpaqueFunction(function=launch_setup)
    ])
