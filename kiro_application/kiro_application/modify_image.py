import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image

class ModifyImage(Node):
    def __init__(self):
        super().__init__('modify_image')

        self.tmp_image = Image()
        self.image_sub_ = self.create_subscription(
            Image,
            'camera/color/image_raw',
            self.image_callback,
            10
        )

        self.image_pub_ = self.create_publisher(
            Image,
            'modify_image',
            10
        )

        self.timer = self.create_timer(0.1, self.timer_callback)
    
    def image_callback(self, msg):
        self.tmp_image = msg

    def timer_callback(self):
        self.image_pub_.publish(self.tmp_image)

def main(args=None):
    rclpy.init(args=args)
    mi = ModifyImage()

    rclpy.spin(mi)

    mi.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()