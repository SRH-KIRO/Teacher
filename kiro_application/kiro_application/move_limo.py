import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist

class MoveLimo(Node):
    def __init__(self):
        super().__init__('move_limo')
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)
    
    def timer_callback(self):
        cmd = Twist()
        cmd.linear.x = 0.3
        cmd.angular.z = 0.3
        self.cmd_pub.publish(cmd)

def main(args=None):
    rclpy.init()
    ml = MoveLimo()

    rclpy.spin(ml)

    ml.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()