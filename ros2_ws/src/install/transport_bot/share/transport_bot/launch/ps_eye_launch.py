from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='v4l2_camera',
            executable='v4l2_camera_node',
            name='playstation_eye',
            parameters=[
                {'video_device': '/dev/video2'},  # Set this to your PlayStation Eye (check with `v4l2-ctl --list-devices`)
                {'image_size': [640, 480]},
                {'image_format': 'yuyv'},
                {'output_encoding': 'rgb8'},
                {'camera_frame_id': 'playstation_eye_frame'}
            ],
            remappings=[
                ('image_raw', '/camera/image_raw'),
                ('camera_info', '/camera/camera_info')
            ],
            output='screen'
        )
    ])
