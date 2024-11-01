from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Запуск черепашки с уровнем логирования WARN
        Node(
            package='turtlesim',
            executable='turtlesim_node',
            name='turtle1',
            output='screen',
            parameters=[{'log_level': 'WARN'}]  
        ),
        
        Node(
            package='turtlesim',
            executable='turtlesim_node',
            name='turtle2',
            output='screen',
            parameters=[{'log_level': 'WARN'}]  
        ),

        Node(
            package='turtlesim',
            executable='turtlesim_node',
            name='turtle3',
            output='screen',
            parameters=[{'log_level': 'WARN'}]  
        ),

        # Узел mimic для turtle2 с уровнем логирования WARN
        Node(
            package='turtlesim',
            executable='mimic',
            name='mimic_turtle2',
            output='screen',
            parameters=[{'log_level': 'WARN'}],  
            remappings=[
                ('/input', '/turtle1'),
                ('/output', '/turtle2')
            ]
        ),

        # Узел mimic для turtle3 с уровнем логирования WARN
        Node(
            package='turtlesim',
            executable='mimic',
            name='mimic_turtle3',
            output='screen',
            parameters=[{'log_level': 'WARN'}], 
            remappings=[
                ('/input', '/turtle2'),
                ('/output', '/turtle3')
            ]
        ),
    ])

