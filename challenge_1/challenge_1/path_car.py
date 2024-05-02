import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose2D
import numpy as np

def calcular_distancia(punto1_x, punto1_y, punto2_x, punto2_y):
    """Calcula la distancia entre dos puntos en un plano cartesiano."""
    distancia = np.sqrt(pow((punto2_x - punto1_x), 2) + pow((punto2_y - punto1_y), 2))
    return distancia

class Car_Line(Node):    
    def __init__(self):
        super().__init__('path_node')

        #Declaracion de Parametros del params
        #PATH 1
        self.declare_parameter('path1.wp', rclpy.Parameter.Type.INTEGER)
        self.declare_parameter('path1.wp1_x', rclpy.Parameter.Type.DOUBLE)
        self.declare_parameter('path1.wp1_y', rclpy.Parameter.Type.DOUBLE)
        self.declare_parameter('path1.wp2_x', rclpy.Parameter.Type.DOUBLE)
        self.declare_parameter('path1.wp2_y', rclpy.Parameter.Type.DOUBLE)
        self.declare_parameter('path1.wp3_x', rclpy.Parameter.Type.DOUBLE)
        self.declare_parameter('path1.wp3_y', rclpy.Parameter.Type.DOUBLE)
        self.declare_parameter('path1.wp4_x', rclpy.Parameter.Type.DOUBLE)
        self.declare_parameter('path1.wp4_y', rclpy.Parameter.Type.DOUBLE)
        self.declare_parameter('path1.wp5_x', rclpy.Parameter.Type.DOUBLE)
        self.declare_parameter('path1.wp5_y', rclpy.Parameter.Type.DOUBLE)
        #PATH 2
        self.declare_parameter('path2.wp', rclpy.Parameter.Type.INTEGER)
        self.declare_parameter('path2.wp1_x', rclpy.Parameter.Type.DOUBLE)
        self.declare_parameter('path2.wp1_y', rclpy.Parameter.Type.DOUBLE)
        self.declare_parameter('path2.wp2_x', rclpy.Parameter.Type.DOUBLE)
        self.declare_parameter('path2.wp2_y', rclpy.Parameter.Type.DOUBLE)
        self.declare_parameter('path2.wp3_x', rclpy.Parameter.Type.DOUBLE)
        self.declare_parameter('path2.wp3_y', rclpy.Parameter.Type.DOUBLE)
        #PATH 3
        self.declare_parameter('path3.wp', rclpy.Parameter.Type.INTEGER)
        self.declare_parameter('path3.wp1_x', rclpy.Parameter.Type.DOUBLE)
        self.declare_parameter('path3.wp1_y', rclpy.Parameter.Type.DOUBLE)
        self.declare_parameter('path3.wp2_x', rclpy.Parameter.Type.DOUBLE)
        self.declare_parameter('path3.wp2_y', rclpy.Parameter.Type.DOUBLE)
        self.declare_parameter('path3.wp3_x', rclpy.Parameter.Type.DOUBLE)
        self.declare_parameter('path3.wp3_y', rclpy.Parameter.Type.DOUBLE)
        self.declare_parameter('path3.wp4_x', rclpy.Parameter.Type.DOUBLE)
        self.declare_parameter('path3.wp4_y', rclpy.Parameter.Type.DOUBLE)
        self.declare_parameter('path3.wp5_x', rclpy.Parameter.Type.DOUBLE)
        self.declare_parameter('path3.wp5_y', rclpy.Parameter.Type.DOUBLE)
        #PATH 4
        self.declare_parameter('path4.wp', rclpy.Parameter.Type.INTEGER)
        self.declare_parameter('path4.wp1_x', rclpy.Parameter.Type.DOUBLE)
        self.declare_parameter('path4.wp1_y', rclpy.Parameter.Type.DOUBLE)
        self.declare_parameter('path4.wp2_x', rclpy.Parameter.Type.DOUBLE)
        self.declare_parameter('path4.wp2_y', rclpy.Parameter.Type.DOUBLE)
        self.declare_parameter('path4.wp3_x', rclpy.Parameter.Type.DOUBLE)
        self.declare_parameter('path4.wp3_y', rclpy.Parameter.Type.DOUBLE)
        self.declare_parameter('path4.wp4_x', rclpy.Parameter.Type.DOUBLE)
        self.declare_parameter('path4.wp4_y', rclpy.Parameter.Type.DOUBLE)
        self.declare_parameter('path4.wp5_x', rclpy.Parameter.Type.DOUBLE)
        self.declare_parameter('path4.wp5_y', rclpy.Parameter.Type.DOUBLE)
        self.declare_parameter('path4.wp6_x', rclpy.Parameter.Type.DOUBLE)
        self.declare_parameter('path4.wp6_y', rclpy.Parameter.Type.DOUBLE)
        self.declare_parameter('path4.wp7_x', rclpy.Parameter.Type.DOUBLE)
        self.declare_parameter('path4.wp7_y', rclpy.Parameter.Type.DOUBLE)
        self.declare_parameter('path4.wp8_x', rclpy.Parameter.Type.DOUBLE)
        self.declare_parameter('path4.wp8_y', rclpy.Parameter.Type.DOUBLE)
        self.declare_parameter('path4.wp9_x', rclpy.Parameter.Type.DOUBLE)
        self.declare_parameter('path4.wp9_y', rclpy.Parameter.Type.DOUBLE)
        self.declare_parameter('path4.wp10_x', rclpy.Parameter.Type.DOUBLE)
        self.declare_parameter('path4.wp10_y', rclpy.Parameter.Type.DOUBLE)
        self.declare_parameter('path4.wp11_x', rclpy.Parameter.Type.DOUBLE)
        self.declare_parameter('path4.wp11_y', rclpy.Parameter.Type.DOUBLE)
        self.declare_parameter('path4.wp12_x', rclpy.Parameter.Type.DOUBLE)
        self.declare_parameter('path4.wp12_y', rclpy.Parameter.Type.DOUBLE)

        #TImer
        timer_period = 0.01
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.get_logger().info('Talker node successfully initialized!! :P')

        self.x_target = 0.0
        self.y_target = 0.0

        self.x_pose = 0.0
        self.y_pose = 0.0

        self.current_path = 0
        self.current_wp = 0

        self.distancia = 0.0

        self.publisher = self.create_publisher(Pose2D, 'Point', 10)
        self.target = Pose2D()

        self.sub = self.create_subscription(Pose2D, 'odom', self.listener_callback, 10)
        self.pose = Pose2D()
        self.get_logger().info('Listener Node initialized!!')

    def timer_callback(self):
        num_wp = 0

        if self.current_path == 0:
            if self.current_wp == 0:
                self.x_target = self.get_parameter('path1.wp1_x').get_parameter_value().double_value
                self.y_target = self.get_parameter('path1.wp1_y').get_parameter_value().double_value
            elif self.current_wp == 1:
                self.x_target = self.get_parameter('path1.wp2_x').get_parameter_value().double_value
                self.y_target = self.get_parameter('path1.wp2_y').get_parameter_value().double_value
            elif self.current_wp == 2:
                self.x_target = self.get_parameter('path1.wp3_x').get_parameter_value().double_value
                self.y_target = self.get_parameter('path1.wp3_y').get_parameter_value().double_value
            elif self.current_wp == 3:
                self.x_target = self.get_parameter('path1.wp4_x').get_parameter_value().double_value
                self.y_target = self.get_parameter('path1.wp4_y').get_parameter_value().double_value
            elif self.current_wp == 4:
                self.x_target = self.get_parameter('path1.wp5_x').get_parameter_value().double_value
                self.y_target = self.get_parameter('path1.wp5_y').get_parameter_value().double_value
            num_wp = 5
            """""
            elif self.current_wp == 5:
                self.current_wp = 0
                self.current_path += 1
            """""
            self.get_logger().info('Parameters declared!')
        elif self.current_path == 1:
            if self.current_wp == 0:
                self.x_target = self.get_parameter('path2.wp1_x').get_parameter_value().double_value
                self.y_target = self.get_parameter('path2.wp1_y').get_parameter_value().double_value
            elif self.current_wp == 1:
                self.x_target = self.get_parameter('path2.wp2_x').get_parameter_value().double_value
                self.y_target = self.get_parameter('path2.wp2_y').get_parameter_value().double_value
            elif self.current_wp == 2:
                self.x_target = self.get_parameter('path2.wp3_x').get_parameter_value().double_value
                self.y_target = self.get_parameter('path2.wp3_y').get_parameter_value().double_value
            num_wp = 3
            """""
            elif self.current_wp == 3:
                self.current_wp = 0
                self.current_path += 1
            """""
            self.get_logger().info('Parameters declared!')
        elif self.current_path == 2:
            if self.current_wp == 0:
                self.x_target = self.get_parameter('path3.wp1_x').get_parameter_value().double_value
                self.y_target = self.get_parameter('path3.wp1_y').get_parameter_value().double_value
            elif self.current_wp == 1:
                self.x_target = self.get_parameter('path3.wp2_x').get_parameter_value().double_value
                self.y_target = self.get_parameter('path3.wp2_y').get_parameter_value().double_value
            elif self.current_wp == 2:
                self.x_target = self.get_parameter('path3.wp3_x').get_parameter_value().double_value
                self.y_target = self.get_parameter('path3.wp3_y').get_parameter_value().double_value
            elif self.current_wp == 3:
                self.x_target = self.get_parameter('path3.wp4_x').get_parameter_value().double_value
                self.y_target = self.get_parameter('path3.wp4_y').get_parameter_value().double_value
            elif self.current_wp == 4:
                self.x_target = self.get_parameter('path3.wp5_x').get_parameter_value().double_value
                self.y_target = self.get_parameter('path3.wp5_y').get_parameter_value().double_value
            num_wp = 5
            """""
            elif self.current_wp == 5:
                self.current_wp = 0
                self.current_path += 1
            """""
            self.get_logger().info('Parameters declared!')
        elif self.current_path == 3:
            if self.current_wp == 0:
                self.x_target = self.get_parameter('path4.wp1_x').get_parameter_value().double_value
                self.y_target = self.get_parameter('path4.wp1_y').get_parameter_value().double_value
            elif self.current_wp == 1:
                self.x_target = self.get_parameter('path4.wp2_x').get_parameter_value().double_value
                self.y_target = self.get_parameter('path4.wp2_y').get_parameter_value().double_value
            elif self.current_wp == 2:
                self.x_target = self.get_parameter('path4.wp3_x').get_parameter_value().double_value
                self.y_target = self.get_parameter('path4.wp3_y').get_parameter_value().double_value
            elif self.current_wp == 3:
                self.x_target = self.get_parameter('path4.wp4_x').get_parameter_value().double_value
                self.y_target = self.get_parameter('path4.wp4_y').get_parameter_value().double_value
            elif self.current_wp == 4:
                self.x_target = self.get_parameter('path4.wp5_x').get_parameter_value().double_value
                self.y_target = self.get_parameter('path4.wp5_y').get_parameter_value().double_value
            elif self.current_wp == 5:
                self.x_target = self.get_parameter('path4.wp6_x').get_parameter_value().double_value
                self.y_target = self.get_parameter('path4.wp6_y').get_parameter_value().double_value
            elif self.current_wp == 6:
                self.x_target = self.get_parameter('path4.wp7_x').get_parameter_value().double_value
                self.y_target = self.get_parameter('path4.wp7_y').get_parameter_value().double_value
            elif self.current_wp == 7:
                self.x_target = self.get_parameter('path4.wp8_x').get_parameter_value().double_value
                self.y_target = self.get_parameter('path4.wp8_y').get_parameter_value().double_value
            elif self.current_wp == 8:
                self.x_target = self.get_parameter('path4.wp9_x').get_parameter_value().double_value
                self.y_target = self.get_parameter('path4.wp9_y').get_parameter_value().double_value
            elif self.current_wp == 9:
                self.x_target = self.get_parameter('path4.wp10_x').get_parameter_value().double_value
                self.y_target = self.get_parameter('path4.wp10_y').get_parameter_value().double_value
            elif self.current_wp == 10:
                self.x_target = self.get_parameter('path4.wp11_x').get_parameter_value().double_value
                self.y_target = self.get_parameter('path4.wp11_y').get_parameter_value().double_value
            elif self.current_wp == 11:
                self.x_target = self.get_parameter('path4.wp12_x').get_parameter_value().double_value
                self.y_target = self.get_parameter('path4.wp12_y').get_parameter_value().double_value
            num_wp = 12
            """""
            elif self.current_wp == 12:
                self.current_wp = 0
                self.current_path += 1
            """""
            self.get_logger().info('Parameters declared!')

        self.x_pose = self.pose.x
        self.y_pose = self.pose.y

        self.target.x = self.x_target
        self.target.y = self.y_target
        self.target.theta = float(self.current_wp)

        self.distancia = calcular_distancia(self.x_pose, self.y_pose, self.x_target, self.y_target)

        if self.distancia <= 0.05:
            self.current_wp += 1
            if self.current_wp >= num_wp:
                self.current_wp = 0
                self.current_path += 1
        
        self.publisher.publish(self.target)

    def listener_callback(self, msg):
        self.pose.x = msg.x
        self.pose.y = msg.y

def main(args=None):
    rclpy.init(args=args)
    m_p = Car_Line()
    rclpy.spin(m_p)
    m_p.destroy_node() 
    rclpy.shutdown()

if __name__ == '__main__':
    main()