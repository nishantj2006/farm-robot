from launch import LaunchDescription
from launch.actions import TimerAction
from launch_ros.actions import ComposableNodeContainer, LoadComposableNodes
from launch_ros.descriptions import ComposableNode

def generate_launch_description():
    # 1. Define the Container
    cam_container = ComposableNodeContainer(
        name='cam_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container_mt',
        output='screen'
    )

    # ---------------- LEFT CAMERA BLOCK ----------------
    left_cam = ComposableNode(
        name='left_cam',
        package='isaac_ros_argus_camera',
        plugin='nvidia::isaac_ros::argus::ArgusMonoNode',
        parameters=[{
            'camera_id': 0,
            'module_id': 0,
        }],
        remappings=[
            ('left/image_raw', '/left/image_raw_native'),
            ('left/camera_info', '/left/camera_info_native')
        ]
    )

    left_resize = ComposableNode(
        name='left_resize',
        package='isaac_ros_image_proc',
        plugin='nvidia::isaac_ros::image_proc::ResizeNode',
        parameters=[{
            'output_width': 1280,
            'output_height': 720,
            'keep_aspect_ratio': False
        }],
        remappings=[
            ('image', '/left/image_raw_native'),
            ('camera_info', '/left/camera_info_native'),
            ('resize/image', '/left/image_raw'),
            ('resize/camera_info', '/left/camera_info')
        ]
    )

    # ---------------- RIGHT CAMERA BLOCK ----------------
    right_cam = ComposableNode(
        name='right_cam',
        package='isaac_ros_argus_camera',
        plugin='nvidia::isaac_ros::argus::ArgusMonoNode',
        parameters=[{
            'camera_id': 1,
            'module_id': 0,
        }],
        remappings=[
            ('left/image_raw', '/right/image_raw_native'),
            ('left/camera_info', '/right/camera_info_native')
        ]
    )

    right_resize = ComposableNode(
        name='right_resize',
        package='isaac_ros_image_proc',
        plugin='nvidia::isaac_ros::image_proc::ResizeNode',
        parameters=[{
            'output_width': 1280,
            'output_height': 720,
            'keep_aspect_ratio': False
        }],
        remappings=[
            ('image', '/right/image_raw_native'),
            ('camera_info', '/right/camera_info_native'),
            ('resize/image', '/right/image_raw'),
            ('resize/camera_info', '/right/camera_info')
        ]
    )

    # ---------------- LOADING & DELAYS ----------------
    # Load Left Camera & Left Resize together
    load_left_nodes = LoadComposableNodes(
        target_container='cam_container',
        composable_node_descriptions=[left_cam, left_resize]
    )

    # Load Right Camera & Right Resize together
    load_right_nodes = LoadComposableNodes(
        target_container='cam_container',
        composable_node_descriptions=[right_cam, right_resize]
    )

    # Staggered startups to prevent libargus crashes
    delay_left = TimerAction(
        period=1.0,
        actions=[load_left_nodes]
    )

    delay_right = TimerAction(
        period=3.0,
        actions=[load_right_nodes]
    )

    return LaunchDescription([
        cam_container,
        delay_left,
        delay_right
    ])
