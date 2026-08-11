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

    # 2. Define Left Camera
    left_cam = ComposableNode(
        name='left_cam',
        package='isaac_ros_argus_camera',
        plugin='nvidia::isaac_ros::argus::ArgusMonoNode',
        parameters=[{
            'camera_id': 0,
            'module_id': 0,
            'image_width': 1280,
            'image_height': 720,
            'fps': 30
        }],
        remappings=[
            ('image_raw', '/left/image_raw'),
            ('camera_info', '/left/camera_info')
        ]
    )

    # 3. Define Right Camera
    right_cam = ComposableNode(
        name='right_cam',
        package='isaac_ros_argus_camera',
        plugin='nvidia::isaac_ros::argus::ArgusMonoNode',
        parameters=[{
            'camera_id': 1,
            'module_id': 0,
            'image_width': 1280,
            'image_height': 720,
            'fps': 30
        }],
        remappings=[
            ('image_raw', '/right/image_raw'),
            ('camera_info', '/right/camera_info')
        ]
    )

    # 4. Action to load the nodes into the container
    load_nodes = LoadComposableNodes(
        target_container='cam_container',
        composable_node_descriptions=[left_cam, right_cam]
    )

    # 5. Wait 2 seconds after container starts before loading nodes (Eliminates Race Condition)
    delayed_node_load = TimerAction(
        period=2.0,
        actions=[load_nodes]
    )

    return LaunchDescription([
        cam_container,
        delayed_node_load
    ])
