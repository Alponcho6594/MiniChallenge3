import rclpy
import rclpy.clock
from rclpy.node import Node
from std_msgs.msg import Float32
from geometry_msgs.msg import Pose2D
import numpy as np

def truncated_remainder(dividend, divisor):
    divided_number = dividend / divisor
    divided_number = \
        -int(-divided_number) if divided_number < 0 else int(divided_number)

    remainder = dividend - divisor * divided_number

    return remainder

def transform_to_pipi(input_angle):
    p1 = truncated_remainder(input_angle + np.sign(input_angle) * np.pi, 2 * np.pi)
    p2 = (np.sign(np.sign(input_angle)
                  + 2 * (np.sign(np.fabs((truncated_remainder(input_angle + np.pi, 2 * np.pi))
                                      / (2 * np.pi))) - 1))) * np.pi

    output_angle = p1 - p2

    return output_angle


class Car_Line(Node):
    def __init__(self):
        super().__init__('odometry_node')
        self.publisher_pose = self.create_publisher(Pose2D, 'odom', 10)

        #dt
        self.timer_period = 0.01
        
        #Velocidad de los Encoder
        self.velocidadLeft = 0.0
        self.velocidadRight = 0.0
        
        #Distancia lineal y angular recorrida
        self.distance = 0.0
        self.angular_distance = 0.0
        
        #Pose actual del robot
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        #Auxiliar Variables
        self.angle = 0.0

        self.timer = self.create_timer(self.timer_period, self.timer_callback)
        self.get_logger().info('Talker node successfully initialized!! :P')

        self.publisher_pose = self.create_publisher(Pose2D, 'odom', 10)
 
        self.pose = Pose2D()

        # Set QoS profile to match the publisher's QoS
        qos_profile = rclpy.qos.qos_profile_sensor_data
        self.sub = self.create_subscription(Float32, 'VelocityEncL', self.listener_callback_left, qos_profile)
        self.sub = self.create_subscription(Float32, 'VelocityEncR', self.listener_callback_right, qos_profile)
        self.get_logger().info('Listener Node initialized!!')

    def timer_callback(self):

        # Actualizamos distancias recorridas
        self.distance = 0.05 * ((self.velocidadLeft + self.velocidadRight) / 2)
        self.angular_distance = 0.05 * ((self.velocidadRight - self.velocidadLeft) / 0.19)
        
        #Actualizamos Pose Actual
        self.angle = self.angle + self.angular_distance * self.timer_period * ((2*np.pi)/5.75)
        self.theta = transform_to_pipi(self.angle)
        self.x = self.x + self.distance * np.cos(self.theta) * self.timer_period
        self.y = self.y + self.distance * np.sin(self.theta) * self.timer_period

        #Publicamos Pose2D
        self.pose.theta = self.theta
        self.pose.x = self.x
        self.pose.y = self.y

        # Publish velocity command
        self.publisher_pose.publish(self.pose)

    def listener_callback_left(self, msg):
        self.velocidadLeft = msg.data

    def listener_callback_right(self, msg):
        self.velocidadRight = msg.data

def main(args=None):
    rclpy.init(args=args)
    m_p = Car_Line()
    rclpy.spin(m_p)
    m_p.destroy_node() 
    rclpy.shutdown()

if __name__ == '__main__':
    main()