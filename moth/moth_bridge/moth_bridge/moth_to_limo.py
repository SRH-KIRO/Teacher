import rclpy

from rclpy.node import Node

from std_msgs.msg import UInt8
from moth_msgs.msg import MothCmd

import websocket
import threading
import json

class MothToLimo(Node):
    def __init__(self):
        super().__init__('moth_to_limo')

        self.channel = "cqjl0h639m9rvs3djuog"
        self.key = "wego"

        self.lift_pub_ = self.create_publisher(UInt8, 'lift_cmd', 10) # lift 조종하는 command
        # moth server에서 limo 시나리오 상테 및 limo control data를 제어하는 부분
        self.moth_cmd_pub_ = self.create_publisher(MothCmd, 'moth_cmd', 10) 
        

        # set initial value for the limo command
        self.moth_cmd_  = MothCmd()
        self.moth_cmd_.scenario_num = MothCmd.DEFAULT
        self.moth_cmd_.control_cmd = MothCmd.STOP

        # set initial value for lift command
        self.lift_cmd_ = UInt8()
        self.lift_cmd_.data = 1 # 1: open,  2: close

        # wall timer for the publisher
        self.wall_timer = self.create_timer(0.1, self.publishData)

        self.ws = websocket.WebSocket()
        self.ws.connect(f"ws://cobot.center:8286/pang/ws/sub?channel={self.channel}&track=control&mode=single&key={self.key}")

        self.thr = threading.Thread(target=self.recv, args=[]).start()
#        self.msg_recver = self.create_timer(0.1, self.recv)

    def recv(self):
        # data 받을 부분
        while True:
            try:
                data = self.ws.recv()
                print("data: ", data)
            except (websocket.WebSocketConnectionClosedException, BrokenPipeError) as e:
                print(f"Error occurred: {e}")
                self.ws.connect(f"ws://cobot.center:8286/pang/ws/sub?channel={self.channel}&track=control&mode=single&key={self.key}")
#                self.ws.connect(f"ws://{host}:{port}/{path}")
#                data = self.ws.recv()
            try:
                jsonObject = json.loads(data)

                scenario_ = jsonObject.get("scenario")
                if scenario_:
                    if scenario_ == "default":
                        self.moth_cmd_.scenario_num = MothCmd.DEFAULT
                    elif scenario_ == "scenario1":
                        self.moth_cmd_.scenario_num = MothCmd.SCENARIOONE
                    elif scenario_ == "scenario2":
                        self.moth_cmd_.scenario_num = MothCmd.SCENARIOTWO

                control_ = jsonObject.get("control")
                if control_:
                    if control_ == "stop":
                        self.moth_cmd_.control_cmd = MothCmd.STOP
                    elif control_ == "forward":
                        self.moth_cmd_.control_cmd = MothCmd.FORWARD
                    elif control_ == "backward":
                        self.moth_cmd_.control_cmd = MothCmd.BACKWARD
                    elif control_ == "turnleft":
                        self.moth_cmd_.control_cmd = MothCmd.TURNLEFT
                    elif control_ == "turnright":
                        self.moth_cmd_.control_cmd = MothCmd.TURNRIGHT

                lift_ = jsonObject.get("lift")
                if lift_ is not None:
#                    print(lift_)
                    if lift_ == 1:
                        self.lift_cmd_.data = 1 # 1: open,  2: close
                    elif lift_ == 0:
                        self.lift_cmd_.data = 2 # 1: open,  2: close

            except Exception as e:
                print(f"error: {e}")


    def setData(self):
        # 변수 업데이트 할 부분
        pass


    def publishData(self):
        self.lift_pub_.publish(self.lift_cmd_)
        self.moth_cmd_pub_.publish(self.moth_cmd_)


def main(args=None):
    rclpy.init(args=args)
    moth_2_limo = MothToLimo()
    rclpy.spin(moth_2_limo)
    moth_2_limo.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
