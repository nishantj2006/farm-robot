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
    
    # Pointing to the new dynamic batched engine
    engine_path = '/workspaces/isaac_ros-dev/src/stereo_depth_yolo/model_batched.plan'

    left_yaml = os.path.join(pkg_dir, 'config', 'left.yaml')
    right_yaml = os.path.join(pkg_dir, 'config', 'right.yaml')

    # ==========================================
    # TRACK A: LEFT CAMERA PIPELINE
    # ==========================================
    left_cam = ComposableNode(
        package='isaac_ros_argus_camera', plugin='nvidia::isaac_ros::argus::ArgusMonoNode',
        name='left_cam', namespace='left',
        parameters=[{'camera_id': 0, 'camera_info_url': f'file://{left_yaml}', 'output_encoding': 'rgb8'}],
        remappings=[('left/image_raw', '/left/image_raw'), ('left/camera_info', '/left/camera_info')]
    )

    left_rectify = ComposableNode(
        package='isaac_ros_image_proc', plugin='nvidia::isaac_ros::image_proc::RectifyNode',
        name='left_rectify', parameters=[{'output_width': width, 'output_height': height}],
        remappings=[
            ('image_raw', '/left/image_raw'), ('camera_info', '/left/camera_info'),
            ('image_rect', '/left/image_rect'), ('camera_info_rect', '/left/camera_info_rect')
        ]
    )

    left_encoder = ComposableNode(
        package='isaac_ros_dnn_image_encoder', plugin='nvidia::isaac_ros::dnn_inference::DnnImageEncoderNode',
        name='left_encoder',
        parameters=[{
            'input_image_width': width,
            'input_image_height': height,
            'network_image_width': width, 
            'network_image_height': net_height, 
            'image_mean': [0.0, 0.0, 0.0], 
            'image_stddev': [1.0, 1.0, 1.0]
        }],
        remappings=[('image', '/left/image_rect'), ('camera_info', '/left/camera_info_rect'), ('encoded_tensor', '/left/encoded_tensor')]
    )

    left_trt = ComposableNode(
        package='isaac_ros_tensor_rt', plugin='nvidia::isaac_ros::dnn_inference::TensorRTNode',
        name='left_trt',
        parameters=[{
            'engine_file_path': engine_path, 
            'force_engine_update': False, 
            'input_tensor_names': ['input_tensor'], 
            'input_binding_names': ['images'], 
            'output_tensor_names': ['output_tensor'], 
            'output_binding_names': ['output0'], 
            'input_dimensions': [1, 3, net_height, width] # Dynamic engine allows batch=1
        }],
        remappings=[('tensor_pub', '/left/encoded_tensor'), ('tensor_sub', '/left/trt_output')]
    )

    left_decoder = ComposableNode(
        package='isaac_ros_yolov8', plugin='nvidia::isaac_ros::yolov8::YoloV8DecoderNode',
        name='left_decoder', parameters=[{'tensor_name': 'output_tensor', 'confidence_threshold': 0.8, 'nms_threshold': 0.45, 'num_classes': 1}],
        remappings=[('tensor_sub', '/left/trt_output'), ('detections_output', '/left/detections')]
    )

    # ==========================================
    # TRACK B: RIGHT CAMERA PIPELINE
    # ==========================================
    right_cam = ComposableNode(
        package='isaac_ros_argus_camera', plugin='nvidia::isaac_ros::argus::ArgusMonoNode',
        name='right_cam', namespace='right',
        parameters=[{'camera_id': 1, 'camera_info_url': f'file://{right_yaml}', 'output_encoding': 'rgb8'}],
        remappings=[('left/image_raw', '/right/image_raw'), ('left/camera_info', '/right/camera_info')]
    )

    right_rectify = ComposableNode(
        package='isaac_ros_image_proc', plugin='nvidia::isaac_ros::image_proc::RectifyNode',
        name='right_rectify', parameters=[{'output_width': width, 'output_height': height}],
        remappings=[
            ('image_raw', '/right/image_raw'), ('camera_info', '/right/camera_info'),
            ('image_rect', '/right/image_rect'), ('camera_info_rect', '/right/camera_info_rect')
        ]
    )

    right_encoder = ComposableNode(
        package='isaac_ros_dnn_image_encoder', plugin='nvidia::isaac_ros::dnn_inference::DnnImageEncoderNode',
        name='right_encoder',
        parameters=[{
            'input_image_width': width,
            'input_image_height': height,
            'network_image_width': width, 
            'network_image_height': net_height, 
            'image_mean': [0.0, 0.0, 0.0], 
            'image_stddev': [1.0, 1.0, 1.0]
        }],
        remappings=[('image', '/right/image_rect'), ('camera_info', '/right/camera_info_rect'), ('encoded_tensor', '/right/encoded_tensor')]
    )

    right_trt = ComposableNode(
        package='isaac_ros_tensor_rt', plugin='nvidia::isaac_ros::dnn_inference::TensorRTNode',
        name='right_trt',
        parameters=[{
            'engine_file_path': engine_path, 
            'input_tensor_names': ['input_tensor'], 
            'input_binding_names': ['images'], 
            'output_tensor_names': ['output_tensor'], 
            'output_binding_names': ['output0'], 
            'input_dimensions': [1, 3, net_height, width] # Dynamic engine allows batch=1
        }],
        remappings=[('tensor_pub', '/right/encoded_tensor'), ('tensor_sub', '/right/trt_output')]
    )

    right_decoder = ComposableNode(
        package='isaac_ros_yolov8', plugin='nvidia::isaac_ros::yolov8::YoloV8DecoderNode',
        name='right_decoder', parameters=[{'tensor_name': 'output_tensor', 'confidence_threshold': 0.5, 'nms_threshold': 0.45, 'num_classes': 1}],
        remappings=[('tensor_sub', '/right/trt_output'), ('detections_output', '/right/detections')]
    )

    container = ComposableNodeContainer(
        name='dual_ai_container', namespace='',
        package='rclcpp_components', executable='component_container_mt',
        composable_node_descriptions=[
            left_cam, left_rectify, left_encoder, left_trt, left_decoder, 
            right_cam, right_rectify, right_encoder, right_trt, right_decoder
        ],
        arguments=['--ros-args', '--log-level', 'error'],
        output='screen'
    )

    spatial_detection_node = Node(
        package='stereo_depth_yolo', executable='spatial_detection_node.py',
        name='spatial_detection_node', output='screen'
    )

    return [container, spatial_detection_node]

def generate_launch_description():
    return LaunchDescription([OpaqueFunction(function=launch_setup)])