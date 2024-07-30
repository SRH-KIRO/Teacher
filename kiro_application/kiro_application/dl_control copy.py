import rclpy
from rclpy.node import Node

from kiro_msgs.msg import Error
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool

from kiro_application.state import MissionNum

from kiro_msgs.msg import BoundingBoxes

from builtin_interfaces.msg import Duration
import time

MAX_STEERING = 1.5

class DlControl(Node):
    def __init__(self):
        super().__init__('dl_control')

        self.stop_flag = True
        self.start_flag = False
        self.control_1 = 0.0
        self.control_2 = 0.0

        self.error_sub_ = self.create_subscription(
            Error,
            'dl_gap',
            self.error_callback,
            10
        )

        self.stop_sub_ = self.create_subscription(
            Bool,
            'stop',
            self.stop_callback,
            10
        )

        self.cmd_pub_ = self.create_publisher(
            Twist,
            'cmd_vel',
            10
        )
    
        self.box_sub_ = self.create_subscription(
            BoundingBoxes,
            'yolo_box',
            self.yolo_callback,
            10
        )

        self.timer_ = self.create_timer(0.1, self.timer_callback)
        self.mission_num = MissionNum.OUTERLINE
        
        self.slow_down_flag = False
        self.turn_right_flag = False
        self.pedestrian_flag = False

        self.slow_down_duration = Duration()
        self.slow_down_duration.sec = 1

        self.turn_right_duration = Duration()
        self.turn_right_duration.sec = 1

    def yolo_callback(self,msg):
        tmp_slow_down = False
        tmp_turn_right = False
        tmp_pedestrian = False

        if msg.boxes:
            for i in msg.boxes:
                if i.name == "slow_down":
                    tmp_slow_down = True
                if i.name == "turn_right":
                    tmp_turn_right = True
                if i.name == "pedestrian":
                    tmp_pedestrian = True
                
    def stop_callback(self, msg):
        self.stop_flag = msg.data

    def error_callback(self, msg):
        self.control_1 =  msg.error_1 * 0.008
        self.control_2 =  msg.error_2 * 0.015
        self.start_flag = True

    def timer_callback(self):
        cmd = Twist()

        if self.start_flag:
            if self.mission_num == MissionNum.OUTERLINE:
                cmd.linear.x = 0.35
                cmd.angular.z = self.control_1
            elif self.mission_num == MissionNum.INNERLINE:
                cmd.linear.x = 0.2
                cmd.angular.z = self.control_2
            else:
                pass

        if self.stop_flag:
            cmd.linear.x = 0.0
            cmd.angular.z =0.0

        cmd.angular.z = min(max(cmd.angular.z, -MAX_STEERING), MAX_STEERING)

        self.cmd_pub_.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    dlc = DlControl()

    rclpy.spin(dlc)

    dlc.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
