# Importa la clase principal para definir lanzamientos en ROS 2
from launch import LaunchDescription

# Importa la acción Node para lanzar nodos ROS 2
from launch_ros.actions import Node

# Permite obtener la ruta del directorio share de un paquete instalado
from ament_index_python.packages import get_package_share_directory

# Módulo estándar para trabajar con rutas de archivos
import os

# Función principal requerida por ROS 2 para ejecutar este archivo de lanzamiento
def generate_launch_description():
    # Construye la ruta completa del archivo URDF dentro del paquete
    urdf_file = os.path.join(
        get_package_share_directory('mi_pkg_python'),  # Paquete que contiene el URDF
        'urdf',
        'ensamblaje.urdf'
    )

    # Devuelve la lista de nodos a lanzar
    return LaunchDescription([

        # Nodo que publica el URDF en el topic /robot_description
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[{'robot_description': open(urdf_file).read()}]
        ),

        # Nodo que abre una interfaz gráfica con sliders para mover las juntas
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
            output='screen'
        ),

        # Nodo que lanza RViz2 para visualizar el robot
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen'
        ),

        # Nodo que agrega una transformación estática: world → base_link
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_tf_pub',
            arguments=['0.10', '0.10', '0.0',  # x y z (en metros)
                       '0', '0', '0',        # roll pitch yaw (en radianes)
                       'world', 'base_link'], # parent frame, child frame
            output='screen'
        )
    ])
