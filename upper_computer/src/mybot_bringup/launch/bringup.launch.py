import launch
import launch_ros
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    mybot_bringup_dir = get_package_share_directory(
        'mybot_bringup')
    rplidar_ros2_dir = get_package_share_directory(
        'rplidar_ros')

    urdf2tf = launch.actions.IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [mybot_bringup_dir, '/launch', '/urdf2tf.launch.py']),
    ) #运行urdf2tf.launch.py，将URDF模型转换为TF坐标系

    odom2tf = launch_ros.actions.Node(
        package='mybot_bringup',
        executable='odom2tf',
        output='screen'
    ) #运行odom2tf节点，将里程计数据转换为TF坐标系

    microros_agent = launch_ros.actions.Node(
        package='micro_ros_agent',
        executable='micro_ros_agent',
        arguments=['serial','--dev','/dev/ttyUSB0','--baudrate','115200'],
        output='screen'
    ) #运行micro_ros_agent节点，连接到Micro-ROS设备,默认使用USB0端口


    rplidar = launch.actions.IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [rplidar_ros2_dir, '/launch', '/view_rplidar_a1_launch.py']),
     ) #运行rplidar的launch文件，启动RPLIDAR传感器的ROS2节点
    
    return launch.LaunchDescription([
        urdf2tf,
        odom2tf,
        microros_agent,
        rplidar
    ]) #返回一个LaunchDescription对象，包含所有需要启动的节点和launch文件