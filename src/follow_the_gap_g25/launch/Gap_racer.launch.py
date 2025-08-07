from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
import os

def generate_launch_description():
    # Ruta al mundo personalizado
    world_path = os.path.expanduser('~/ros2_ws_2502/src/f112th_sim_2502_x_ray/worlds/Carrera.world')

    # Lanzar simulación incluyendo otro launch
    sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('f112th_sim_2502_x_ray'),
                'launch',
                'launch_sim.launch.py'
            ])
        ]),
        launch_arguments={'world': world_path}.items()
    )

    # Ejecutar nodos con ros2 run
    nodo_carrera = ExecuteProcess(
        cmd=['ros2', 'run', 'follow_the_gap_g25', 'Gap'],
        output='screen'
    )


    return LaunchDescription([
        sim_launch,
        nodo_carrera,
    ])