import os
import sys
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import TimerAction, OpaqueFunction, ExecuteProcess
from launch_ros.actions import ComposableNodeContainer, PushRosNamespace, Node
from launch_ros.descriptions import ComposableNode

# 获取包路径
pkg_bringup_share = get_package_share_directory('light_bringup')
sys.path.append(os.path.join(pkg_bringup_share, 'launch'))

def generate_launch_description():
    # 加载配置文件
    launch_params_path = os.path.join(pkg_bringup_share, 'config', 'launch_params.yaml')
    with open(launch_params_path, 'r') as f:
        launch_params = yaml.safe_load(f)
    
    namespace = launch_params.get('namespace', '')
    video_play = launch_params.get('video_play', False)  # 读取视频标志位
    
    # 参数文件路径生成函数
    def get_params(param_file_name):
        return os.path.join(
            get_package_share_directory('light_bringup'), 
            'config', 'node_params', 
            f'{param_file_name}_params.yaml'
        )
    
    # 1. 串口节点
    serial_node = ComposableNode(
        package='light_serial',
        plugin='light_serial::UARTNode',
        name='light_serial',
        parameters=[get_params('serial')],
        extra_arguments=[{'use_intra_process_comms': True}]
    )
    
    serial_container = ComposableNodeContainer(
        name='serial_container',
        namespace=namespace,
        package='rclcpp_components',
        executable='component_container_mt',
        composable_node_descriptions=[serial_node],
        output='both',
        emulate_tty=True,
    )

    # 2. 解算节点
    solver_node = ComposableNode(
        package='light_solver',
        plugin='light_solver::SolverNode',
        name='light_solver',
        parameters=[get_params('solver')],
        extra_arguments=[{'use_intra_process_comms': True}]
    )
    
    solver_container = ComposableNodeContainer(
        name='solver_container',
        namespace=namespace,
        package='rclcpp_components',
        executable='component_container_mt',
        composable_node_descriptions=[solver_node],
        output='both',
        emulate_tty=True,
    )

    # 3. 识别节点（Python版本，替换C++组件）
    detector_node = Node(
        package='light_detector',
        executable='detector_node',
        name='light_detector',
        parameters=[get_params('detector')],
        output='both',
        emulate_tty=True,
    )

    # 动态生成启动项
    def generate_launch_items(context, *args, **kwargs):
        launch_items = [
            PushRosNamespace(namespace),
            
            # 1. 先启动串口节点
            TimerAction(
                period=0.5,
                actions=[serial_container]
            ),
            
            # 2. 启动识别节点（Python节点）
            TimerAction(
                period=1.5,
                actions=[detector_node]
            ),
            
            # 3. 最后启动解算节点
            TimerAction(
                period=2.0,
                actions=[solver_container]
            )
        ]
        
        # 如果不读取视频，才启动相机节点
        if not video_play:
            # 相机节点
            camera_node = ComposableNode(
                package='light_camera',
                plugin='light_camera::HikCameraNode',
                name='light_camera',
                parameters=[get_params('camera')],
                extra_arguments=[{'use_intra_process_comms': True}]
            )
            
            camera_container = ComposableNodeContainer(
                name='camera_container',
                namespace=namespace,
                package='rclcpp_components',
                executable='component_container_mt',
                composable_node_descriptions=[camera_node],
                output='both',
                emulate_tty=True,
            )
            
            # 添加相机节点启动项
            launch_items.insert(1, TimerAction(
                period=1.0,
                actions=[camera_container]
            ))
        
        return launch_items
    
    return LaunchDescription([
        OpaqueFunction(function=generate_launch_items)
    ])
