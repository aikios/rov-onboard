from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    # Photogrammetry capture node (triggers Pi Zero camera over HTTP)
    photogrammetry_node = Node(
        package='rov_cameras',
        executable='photogrammetry_node',
        name='photogrammetry_node',
        parameters=[{
            'capture_url': 'http://192.168.7.2:8080/capture',
            'status_url': 'http://192.168.7.2:8080/status',
            'jpeg_quality': 95,
        }],
        output='screen',
    )

    # Pilot camera - forward view
    pilot_cam_front = Node(
        package='usb_cam',
        executable='usb_cam_node_exe',
        name='pilot_cam_front',
        namespace='camera/pilot_front',
        parameters=[{
            'video_device': '/dev/video0',
            'image_width': 640,
            'image_height': 480,
            'framerate': 30.0,
            'pixel_format': 'mjpeg2rgb',
            'camera_frame_id': 'pilot_front_camera',
        }],
        output='screen',
    )

    # Pilot camera - downward view
    pilot_cam_down = Node(
        package='usb_cam',
        executable='usb_cam_node_exe',
        name='pilot_cam_down',
        namespace='camera/pilot_down',
        parameters=[{
            'video_device': '/dev/video2',
            'image_width': 640,
            'image_height': 480,
            'framerate': 30.0,
            'pixel_format': 'mjpeg2rgb',
            'camera_frame_id': 'pilot_down_camera',
        }],
        output='screen',
    )

    return LaunchDescription([
        photogrammetry_node,
        pilot_cam_front,
        pilot_cam_down,
    ])
