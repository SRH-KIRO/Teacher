import rclpy
from rclpy.node import Node

from kiro_msgs.msg import Error
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool

from kiro_application.state import MissionNum

from kiro_msgs.msg import BoundingBoxes

from builtin_interfaces.msg import Duration
import time

import math

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
        self.mission_num = MissionNum.INNERLINE
        
        self.slow_down_flag = False
        self.turn_right_flag = False
        self.pedestrian_flag = False

        self.slow_down_duration = Duration()
        self.slow_down_duration.sec = 1
        self.slow_down_past_time = time.time()

        self.turn_right_duration = Duration()
        self.turn_right_duration.sec = 2
        self.turn_right_past_time = time.time()

        self.pedestrian_size = 200.0
        self.pedestrian_duration = Duration()
        self.pedestrian_duration.sec = 3
        self.pedestrian_duration_pass_by = Duration()
        self.pedestrian_duration_pass_by.sec = 6
        self.pedestrian_start_time = time.time()

    def yolo_callback(self,msg):
        tmp_slow_down = False
        tmp_turn_right = False
        tmp_pedestrian = False

        current_time = time.time()

        if msg.boxes:
            for i in msg.boxes:
                if i.name == "slow_down":
                    tmp_slow_down = True
                    self.slow_down_past_time = current_time
                if i.name == "turn_right":
                    tmp_turn_right = True
                    self.turn_right_past_time = current_time
                if i.name == "pedestrian":
                    if math.sqrt((i.xmin - i.xmax)**2 + (i.ymin - i.ymax)**2) > self.pedestrian_size:
                        tmp_pedestrian = True
        
        if tmp_slow_down:
            self.slow_down_flag = True
        else:
            if int(current_time - self.slow_down_past_time) > self.slow_down_duration.sec:
                self.slow_down_flag = False
            else:
                self.slow_down_flag = True

        if tmp_turn_right:
            self.turn_right_flag = True
        else:
            if int(current_time - self.turn_right_past_time) > self.turn_right_duration.sec:
                self.turn_right_flag = False
            else:
                self.turn_right_flag = True
        
        if tmp_pedestrian:
            if int(current_time - self.pedestrian_start_time) > self.pedestrian_duration_pass_by.sec:
                self.pedestrian_start_time = current_time

        ped_time_duration = int(current_time - self.pedestrian_start_time)

        if ped_time_duration <= self.pedestrian_duration.sec:
            self.pedestrian_flag = True
        elif self.pedestrian_duration_pass_by.sec > ped_time_duration >= self.pedestrian_duration.sec:
            self.pedestrian_flag = False
        else:
            if tmp_pedestrian:
                self.pedestrian_flag = True
            else:
                self.pedestrian_flag = False

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
                if self.slow_down_flag:
                    cmd.linear.x = 0.15
                if self.pedestrian_flag:
                    cmd.linear.x = 0.0
                    cmd.angular.z =0.0
            elif self.mission_num == MissionNum.INNERLINE:
                cmd.linear.x = 0.35
                cmd.angular.z = self.control_1
                if self.slow_down_flag:
                    cmd.linear.x = 0.15
                if self.pedestrian_flag:
                    cmd.linear.x = 0.0
                    cmd.angular.z =0.0
                if self.turn_right_flag:
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
