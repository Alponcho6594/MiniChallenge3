import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose2D
from std_msgs.msg import String
import math

def calcular_distancia(punto1_x, punto1_y, punto2_x, punto2_y):
    """Calcula la distancia entre dos puntos en un plano cartesiano."""
    distancia = math.sqrt(pow((punto2_x - punto1_x), 2) + pow((punto2_y - punto1_y), 2))
    return distancia

def calcular_angulo(punto1_x, punto1_y, punto2_x, punto2_y):
    """Calcula el ángulo necesario para llegar al segundo punto desde el primero."""
    delta_x = punto2_x - punto1_x
    delta_y = punto2_y - punto1_y
    angulo_radianes = math.atan2(delta_y, delta_x)
    return angulo_radianes

class Car_Line(Node):    
    def __init__(self):
        super().__init__('controller_node')
    
        timer_period = 0.01

        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.pub = self.create_publisher(String, 'color_actual', 10)

        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.get_logger().info('Talker node successfully initialized!! :P')
        self.speed_msg = Twist()

        self.current_x = 0.0
        self.current_y = 0.0
        self.current_theta = 0.0

        self.target_x = 0.0
        self.target_y = 0.0
        self.target_theta = 0.0   

        self.target_x_last = 0.0
        self.target_y_last = 0.0

        self.error_theta = 0.0
        self.error_theta_norm = 0.0

        self.flag_spin = 1
        self.error_distancia = 0.0

        self.angular_velocity = 0.0
        self.linear_velocity = 0.0

        self.kp = 0.1
        
        self.sub = self.create_subscription(Pose2D, 'odom', self.listener_callback_odom, 10)
        self.sub = self.create_subscription(Pose2D, 'Point', self.listener_callback_point, 10)
        self.sub = self.create_subscription(String, 'color', self.listener_callback_color, 10)
        
        self.color = 'GREEN'
        self.color_actual = String()
        self.current_pose = Pose2D()
        self.target_pose = Pose2D()
        self.get_logger().info('Listener Node initialized!!')

    def timer_callback(self):

        self.speed_msg.linear.x = 0.0
        self.speed_msg.linear.y = 0.0
        self.speed_msg.linear.z = 0.0
        self.speed_msg.angular.x = 0.0
        self.speed_msg.angular.y = 0.0
        self.speed_msg.angular.z = 0.0

        self.current_x = self.current_pose.x
        self.current_y = self.current_pose.y
        self.current_theta = self.current_pose.theta

        self.target_x = self.target_pose.x
        self.target_y = self.target_pose.y
        self.target_theta = calcular_angulo(self.current_x, self.current_y, self.target_x, self.target_y)

        self.error_theta = self.target_theta - self.current_theta
        self.error_theta_norm = math.atan2(math.sin(self.error_theta), math.cos(self.error_theta))

        self.error_distancia = calcular_distancia(self.current_x, self.current_y, self.target_x, self.target_y)

        """""
        if self.flag_spin == 1:
            if abs(self.error_theta_norm) > 0.05:
                if self.error_theta_norm > 0:
                    self.speed_msg.angular.z = 0.1
                else:
                    self.speed_msg.angular.z = -0.1
            else:
                self.flag_spin = 0
        elif self.flag_spin == 0:
            self.distancia = calcular_distancia(self.current_x, self.current_y, self.target_x, self.target_y)
            if self.distancia > 0.2:
                self.speed_msg.linear.x = 0.05
            else:
                self.flag_spin = 1
        """""
        """""
        if self.flag_spin == 1:
            if abs(self.error_theta_norm) > 0.05:
                self.angular_velocity = self.kp * self.error_theta_norm
                if self.angular_velocity > 0:
                    self.angular_velocity = max(min(self.angular_velocity, 0.3), 0.05)  # Limitar el valor dentro del rango permitido para positivos
                else:
                    self.angular_velocity = min(max(self.angular_velocity, -0.3), -0.05)  # Limitar el valor dentro del rango permitido para negativos
                self.speed_msg.angular.z = self.angular_velocity
            else:
                self.flag_spin = 0
        elif self.flag_spin == 0:
            if self.error_distancia > 0.2:
                self.linear_velocity = self.kp * self.error_distancia
                self.linear_velocity = max(min(self.linear_velocity, 0.3), 0.05)
                self.speed_msg.linear.x = self.linear_velocity
            else:
                self.flag_spin = 1    
        """""
       
        if abs(self.error_theta_norm) > 0.05:
            if self.color == 'RED':
                self.angular_velocity = 0.0
            elif self.color == 'YELLOW':
                self.angular_velocity = self.kp * self.error_theta_norm * 0.5
                if self.angular_velocity > 0:
                    self.angular_velocity = max(min(self.angular_velocity, 0.3), 0.03)  # Limitar el valor dentro del rango permitido para positivos
                else:
                    self.angular_velocity = min(max(self.angular_velocity, -0.3), -0.03)  # Limitar el valor dentro del rango permitido para negativos
            else: 
                self.angular_velocity = self.kp * self.error_theta_norm
                if self.angular_velocity > 0:
                    self.angular_velocity = max(min(self.angular_velocity, 0.3), 0.03)  # Limitar el valor dentro del rango permitido para positivos
                else:
                    self.angular_velocity = min(max(self.angular_velocity, -0.3), -0.03)  # Limitar el valor dentro del rango permitido para negativos
            
            self.speed_msg.angular.z = self.angular_velocity
        if self.error_distancia > 0.05:
            if self.color == 'RED':
                self.linear_velocity = 0.0
            elif self.color == 'YELLOW':
                self.linear_velocity = self.kp * self.error_distancia * 0.5
                self.linear_velocity = max(min(self.linear_velocity, 0.3), 0.03)
            else:
                self.linear_velocity = self.kp * self.error_distancia
                self.linear_velocity = max(min(self.linear_velocity, 0.3), 0.03)
                
            self.speed_msg.linear.x = self.linear_velocity

            self.color_actual.data = self.color

            self.publisher.publish(self.speed_msg)
            self.pub.publish(self.color_actual)
        

        """""
        if self.flag_spin == 1:
            if abs(self.error_theta_norm) > 0.05:
                self.angular_velocity = self.kp * self.error_theta_norm
                if self.angular_velocity > 0:
                    self.angular_velocity = max(min(self.angular_velocity, 0.3), 0.05)  # Limitar el valor dentro del rango permitido para positivos
                else:
                    self.angular_velocity = min(max(self.angular_velocity, -0.3), -0.05)  # Limitar el valor dentro del rango permitido para negativos
                self.speed_msg.angular.z = self.angular_velocity
            else:
                self.flag_spin = 0
        elif self.flag_spin == 0:
            if self.error_distancia > 0.05:
                self.linear_velocity = self.kp * self.error_distancia
                self.linear_velocity = max(min(self.linear_velocity, 0.3), 0.05)
                self.speed_msg.linear.x = self.linear_velocity
                self.flag_spin = 1
        """""

    def listener_callback_odom(self, msg):
        self.current_pose.x = msg.x
        self.current_pose.y = msg.y
        self.current_pose.theta = msg.theta

    def listener_callback_point(self, msg):
        self.target_pose.x = msg.x
        self.target_pose.y = msg.y
        self.target_pose.theta = msg.theta

    def listener_callback_color(self, msg):
        self.color = msg.data

def main(args=None):
    rclpy.init(args=args)
    m_p = Car_Line()
    rclpy.spin(m_p)
    m_p.destroy_node() 
    rclpy.shutdown()

if __name__ == '__main__':
    main()