import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA

class CuboPublisher(Node):
    def __init__(self):
        super().__init__('cubo_publisher')
        self.publisher = self.create_publisher(Marker, 'visualization_marker', 10)
        self.timer = self.create_timer(1.0, self.publicar_cubos)

    def publicar_cubos(self):
        posiciones = [
            (0.1, 0.2, 0.0),
            (0.5, 0.3, 0.0),
            (0.2, 0.3, 0.0)
        ]

        for i, (x, y, z) in enumerate(posiciones):
            cubo = Marker()
            cubo.header.frame_id = 'world'
            cubo.header.stamp = self.get_clock().now().to_msg()
            cubo.ns = 'cubos'
            cubo.id = i
            cubo.type = Marker.CUBE
            cubo.action = Marker.ADD
            cubo.pose.position.x = x
            cubo.pose.position.y = y
            cubo.pose.position.z = z + 0.015  # para que se vea encima del suelo
            cubo.pose.orientation.x = 0.0
            cubo.pose.orientation.y = 0.0
            cubo.pose.orientation.z = 0.0
            cubo.pose.orientation.w = 1.0
            cubo.scale.x = 0.03
            cubo.scale.y = 0.03
            cubo.scale.z = 0.03
            cubo.color = ColorRGBA(r=1.0, g=0.0, b=0.0, a=1.0)
            cubo.lifetime.sec = 0  # 0 = permanente

            self.publisher.publish(cubo)
            self.get_logger().info(f'Cubo {i} publicado en ({x}, {y}, {z})')

def main(args=None):
    rclpy.init(args=args)
    node = CuboPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
