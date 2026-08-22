import launch
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode
import math

def generate_launch_description():

    # 1. Static TF Publishers
    # NEW: Bridges robot base to camera (41.5-degree pitch up)
    pitch_rad = -41.5 * (math.pi / 180.0)
    tf_base = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='tf_base_to_camera',
        arguments=['0', '0', '0', '0', str(pitch_rad), '0', 'base_link', 'camera_link']
    )

    # Bridges camera_link to optical frames
    tf_left = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='tf_left_optical',
        arguments=['0', '0', '0', '-1.5708', '0', '-1.5708', 'camera_link', 'left_camera_optical_frame']
    )

    tf_right = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='tf_right_optical',
        arguments=['0', '-0.06', '0', '-1.5708', '0', '-1.5708', 'camera_link', 'right_camera_optical_frame']
    )
    
    # 2. IMU Driver Node
    # NEW: Reads the ICM20948 from I2C bus 7, address 0x68
    imu_driver = Node(
        package='ros2_icm20948',  
        executable='icm20948_node',
        name='imu_node',
        parameters=[{
            'i2c_bus': 7,
            'i2c_address': 0x68
        }],
        remappings=[
            ('imu/data', '/imu/data')
        ]
    )

    # 3. Spoofer Node Component
    spoofer_component = ComposableNode(
        package='stereo_sync_spoofer',
        plugin='stereo_sync_spoofer::StereoSyncSpoofer',
        name='stereo_sync_spoofer'
    )

    # 4. Isaac ROS Visual SLAM Component
    visual_slam_component = ComposableNode(
        package='isaac_ros_visual_slam',
        plugin='nvidia::isaac_ros::visual_slam::VisualSlamNode',
        name='visual_slam_node',
        parameters=[{
            'enable_imu': True,                     # UPDATED
            'num_cameras': 2,
            'base_frame': 'base_link',              # UPDATED
            'camera_optical_frames': ['left_camera_optical_frame', 'right_camera_optical_frame'],
            'enable_image_denoising': False,
            'enable_rectified_pose': True,
            'image_jitter_threshold_ms': 55.0
        }],
        remappings=[
            ('visual_slam/imu', '/imu/data'),
            ('visual_slam/image_0', '/visual_slam/image_0'),
            ('visual_slam/camera_info_0', '/visual_slam/camera_info_0'),
            ('visual_slam/image_1', '/visual_slam/image_1'),
            ('visual_slam/camera_info_1', '/visual_slam/camera_info_1')
        ]
    )

    # Single container running BOTH components
    pipeline_container = ComposableNodeContainer(
        name='isaac_ros_pipeline_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container_mt',
        composable_node_descriptions=[
            spoofer_component,
            visual_slam_component
        ],
        output='screen'
    )

    return launch.LaunchDescription([
        tf_base,
        tf_left,
        tf_right,
        imu_driver,
        pipeline_container
    ])
