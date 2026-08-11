import os
from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode

def generate_launch_description():
    # 1. Left Rectify Node
    left_rectify_node = ComposableNode(
        name='left_rectify_node',
        package='isaac_ros_image_proc',
        plugin='nvidia::isaac_ros::image_proc::RectifyNode',
        parameters=[{'output_width': 1280, 'output_height': 736}],
        remappings=[
            ('image_raw', '/left/image_raw'),
            ('camera_info', '/left/camera_info'),
            ('image_rect', '/left/image_rect'),
            ('camera_info_rect', '/left/camera_info_rect')
        ]
    )

    # 2. Right Rectify Node
    right_rectify_node = ComposableNode(
        name='right_rectify_node',
        package='isaac_ros_image_proc',
        plugin='nvidia::isaac_ros::image_proc::RectifyNode',
        parameters=[{'output_width': 1280, 'output_height': 736}],
        remappings=[
            ('image_raw', '/right/image_raw'),
            ('camera_info', '/right/camera_info'),
            ('image_rect', '/right/image_rect'),
            ('camera_info_rect', '/right/camera_info_rect')
        ]
    )

    # 3. Disparity Node (Stereo Depth)
    disparity_node = ComposableNode(
        name='disparity_node',
        package='isaac_ros_stereo_image_proc',
        plugin='nvidia::isaac_ros::stereo_image_proc::DisparityNode',
        parameters=[{'max_disparity': 128.0}],
        remappings=[
            ('left/image_rect', '/left/image_rect'),
            ('left/camera_info', '/left/camera_info_rect'),
            ('right/image_rect', '/right/image_rect'),
            ('right/camera_info', '/right/camera_info_rect'),
            ('disparity', '/disparity')
        ]
    )

    # 4. YOLO DNN Image Encoder
    yolo_encoder = ComposableNode(
        name='yolo_encoder',
        package='isaac_ros_dnn_image_encoder',
        plugin='nvidia::isaac_ros::dnn_inference::DnnImageEncoderNode',
        parameters=[{
            'input_image_width': 1280,
            'input_image_height': 736,
            'network_image_width': 1280,
            'network_image_height': 736,
            'image_mean': [0.0, 0.0, 0.0],
            'image_stddev': [1.0, 1.0, 1.0],
        }],
        remappings=[
            ('image', '/left/image_rect'),
            ('camera_info', '/left/camera_info_rect'),
            ('encoded_tensor', '/tensor_pub')
        ]
    )

    # 5. TensorRT Node
    engine_path = '/workspaces/isaac_ros-dev/src/stereo_depth_yolo/model.plan'
    tensorrt_node = ComposableNode(
        name='tensorrt_node',
        package='isaac_ros_tensor_rt',
        plugin='nvidia::isaac_ros::dnn_inference::TensorRTNode',
        parameters=[{
            'model_file_path': '/workspaces/isaac_ros-dev/src/stereo_depth_yolo/best.onnx',
            'engine_file_path': engine_path,
            'force_engine_update': False,
            'input_tensor_names': ['input_tensor'],
            'input_binding_names': ['images'],
            'output_tensor_names': ['output_tensor'],
            'output_binding_names': ['output0'],
        }],
        remappings=[
            ('tensor_pub', '/tensor_pub'),
            ('tensor_sub', '/tensor_sub')
        ]
    )

    # 6. YOLOv8 Decoder Node (Parses TRT tensor output into Detection2DArray)
    yolo_decoder_node = ComposableNode(
        name='yolo_decoder_node',
        package='isaac_ros_yolov8',
        plugin='nvidia::isaac_ros::yolov8::YoloV8DecoderNode',
        parameters=[{
            'confidence_threshold': 0.5,
            'nms_threshold': 0.45,
            'num_classes': 1
        }],
        remappings=[
            ('tensor_sub', '/tensor_sub'),
            ('detections_output', '/detections_output')
        ]
    )

    container = ComposableNodeContainer(
        name='stereo_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container_mt',
        composable_node_descriptions=[
            left_rectify_node,
            right_rectify_node,
            disparity_node,
            yolo_encoder,
            tensorrt_node,
            yolo_decoder_node
        ],
        output='screen'
    )

    camera_driver_node = Node(
        package='stereo_depth_yolo',
        executable='stereo_camera_driver.py',
        name='stereo_camera_driver',
        output='screen'
    )

    spatial_detection_node = Node(
        package='stereo_depth_yolo',
        executable='spatial_detection_node.py',
        name='spatial_detection_node',
        output='screen'
    )

    return LaunchDescription([
        container,
        camera_driver_node,
        spatial_detection_node
    ])
