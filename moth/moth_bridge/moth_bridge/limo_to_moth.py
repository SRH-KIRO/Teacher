import cv2
import numpy as np
from cv_bridge import CvBridge

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Bool

import websocket
import gi
gi.require_version("Gst", "1.0")
from gi.repository import Gst

class LimoToMoth(Node):
    def __init__(self):
        super().__init__('limo_to_moth')

        # CV bridge
        self.br = CvBridge()

        # 기본 이미지 받는 subscription
        self.orignal_img_sub_ = self.create_subscription(
                                        Image,
                                        '/camera/color/image_raw',
                                        self.originalImageCallback,
                                        rclpy.qos.qos_profile_sensor_data
                                    )
        
        # yolo 이미지 받는 subscription
        self.yolo_img_sub_ = self.create_subscription(
                                    Image,
                                    'yolo_image',
                                    self.yoloImageCallback,
                                    10
                                    )
        
        self.emergency_situation_sub_ = self.create_subscription(
                                        Bool,
                                        'emergency_situation',
                                        self.emergencySituationCallback,
                                        10
        )

        self.orignal_img_sub_
        self.yolo_img_sub_
        self.emergency_situation_sub_
    

        Gst.init(None)
        self.cmd_ori = "appsrc do-timestamp=true is-live=true format=time name=src caps=video/x-raw,format=BGR,width=640,height=400 ! videoconvert ! x264enc bitrate=2048 key-int-max=30 speed-preset=1 tune=zerolatency ! video/x-h264, alignment=au, stream-format=byte-stream ! h264parse config-interval=1 ! queue ! appsink name=sink sync=false drop=true max-buffers=2 emit-signals=true"
#        self.cmd_ori = "appsrc name=src caps=video/x-raw,format=BGR,width=640,height=400 ! videoconvert ! jpegenc quality=50 ! jpegparse ! queue ! appsink name=sink sync=false drop=true max-buffers=2 emit-signals=true"
        self.pipeline_ori = Gst.parse_launch(self.cmd_ori)
        self.src_ori = self.pipeline_ori.get_by_name("src")
        self.sink_ori = self.pipeline_ori.get_by_name("sink")
        self.sink_ori.connect("new-sample", self.originalImageSenderCallback)
        self.pipeline_ori.set_state(Gst.State.PLAYING)
#        self.mime_ori = "image/jpeg;width=640;height=400;codecs=jpeg"
        self.mime_ori = "video/h264;width=640;height=400;codecs=avc1.42002A"

        self.cmd_yolo = "appsrc do-timestamp=true is-live=true format=time name=src caps=video/x-raw,format=BGR,width=640,height=400 ! videoconvert ! x264enc bitrate=2048 key-int-max=30 speed-preset=1 tune=zerolatency ! video/x-h264, alignment=au, stream-format=byte-stream ! h264parse config-interval=1 ! queue ! appsink name=sink sync=false drop=true max-buffers=2 emit-signals=true"
#        self.cmd_yolo = "appsrc name=src caps=video/x-raw,format=BGR,width=640,height=400 ! videoconvert ! jpegenc quality=50 ! jpegparse ! queue ! appsink name=sink sync=false drop=true max-buffers=2 emit-signals=true"
        self.pipeline_yolo = Gst.parse_launch(self.cmd_yolo)
        self.src_yolo = self.pipeline_yolo.get_by_name("src")
        self.sink_yolo = self.pipeline_yolo.get_by_name("sink")
        self.sink_yolo.connect("new-sample", self.yoloImageSenderCallback)
        self.pipeline_yolo.set_state(Gst.State.PLAYING)
#        self.mime_yolo = "image/jpeg;width=640;height=400;codecs=jpeg"
        self.mime_yolo = "video/h264;width=640;height=400;codecs=avc1.42002A"


        self.ws_ori = websocket.WebSocket()
        self.ws_ori.connect(f"ws://cobot.center:8286/pang/ws/pub?channel={channel_o}&track=video_o&mode=single&key={key_o}")
        self.ws_yolo = websocket.WebSocket()
        self.ws_yolo.connect(f"ws://cobot.center:8286/pang/ws/pub?channel={channel_y}&track=video_y&mode=single&key={key_y}")
        self.ws_ori.send(self.mime_ori)
        self.ws_yolo.send(self.mime_yolo)
#        self.original_sender = self.create_timer(0.01, self.originalImageSenderCallback)
#        self.yolo_sender = self.create_timer(0.01, self.yoloImageSenderCallback)

    def originalImageCallback(self, msg):
        # 요기에 기본 이미지 받아 publish 하는 부분 작업 하면 됩니다.
        image_ = self.br.imgmsg_to_cv2(msg, 'bgr8')
#        image_ = cv2.cvtColor(self.br.imgmsg_to_cv2(msg, 'bgr8'), cv2.COLOR_BGR2YUV)
        buf = Gst.Buffer.new_wrapped(bytes(image_))
        self.src_ori.emit("push-buffer", buf)


    def yoloImageCallback(self, msg):
        # 요기에는 yolo 이미지 받아 publish 하는 부분 작업 하면 됩니다.
        image_ = self.br.imgmsg_to_cv2(msg, 'bgr8')
        buf = Gst.Buffer.new_wrapped(bytes(image_))
        self.src_yolo.emit("push-buffer", buf)


    def emergencySituationCallback(self, msg):
        # bool data true 일때 빨간색, Bool Data false일 때, 초록색
        pass

    def originalImageSenderCallback(self, appsink):
        sample = self.sink_ori.emit("pull-sample")
        if sample:
            buf = sample.get_buffer()
            image = buf.extract_dup(0, buf.get_size())
            try:
                self.ws_ori.send(image, opcode=websocket.ABNF.OPCODE_BINARY)
            except (websocket.WebSocketConnectionClosedException, BrokenPipeError) as e:
                print(f"Error occurred: {e}")
                self.ws_ori.connect("ws://cobot.center:8286/pang/ws/pub?channel=instant&name=wego&track=video_o&mode=single")
                self.ws_ori.send(self.mime_ori)
#                self.ws_ori.connect(f"ws://{host_o}:{port_o}/{path_o}")
            return Gst.FlowReturn.OK
        return Gst.FlowReturn.Error

    def yoloImageSenderCallback(self, appsink):
        sample = self.sink_yolo.emit("pull-sample")
        if sample:
            buf = sample.get_buffer()
            image = buf.extract_dup(0, buf.get_size())
            try:
                self.ws_yolo.send(image, opcode=websocket.ABNF.OPCODE_BINARY)
            except (websocket.WebSocketConnectionClosedException, BrokenPipeError) as e:
                print(f"Error occurred: {e}")
                self.ws_yolo.connect("ws://cobot.center:8286/pang/ws/pub?channel=instant&name=wego&track=video_y&mode=single")
                self.ws_yolo.send(self.mime_yolo)
#                self.ws_yolo.connect(f"ws://{host_y}:{port_y}/{path_y}")
            return Gst.FlowReturn.OK
        return Gst.FlowReturn.Error

def main(args=None):
    rclpy.init(args=args)
    limo_2_moth = LimoToMoth()
    rclpy.spin(limo_2_moth)
    limo_2_moth.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
