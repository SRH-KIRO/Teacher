import rclpy
from rclpy.node import Node

import cv2
import numpy as np
from cv_bridge import CvBridge
from tensorflow import keras

from sensor_msgs.msg import Image
from kiro_msgs.msg import Error

from ament_index_python.packages import get_package_share_directory
import os

class DLDetectLine(Node):
    def __init__(self):
        super().__init__('dl_detect_line')

        self.br = CvBridge()

        #set default weigh file path
        package_directory = get_package_share_directory('kiro_application')
        folder_name = 'weight'
        file_name = 'dl_drive.h5'
        self.weight = os.path.join(package_directory, folder_name, file_name)

        self.initModel()

        self.subscription = self.create_subscription(
                            Image,
                            'modify_image', 
                            self.image_callback, 
                            10)
        self.subscription # to prevent from warning

        # Publish Image for debugging
        self.debug_publisher = self.create_publisher(Image, 'debug_image', 10)
        self.error_publisher = self.create_publisher(Error, 'dl_gap', 10)


    def initModel(self):
        input1 = keras.layers.Input(
            shape=(
                110,
                300,
                3,
            )
        )

        conv1 = keras.layers.Conv2D(filters=16, kernel_size=(3, 3), strides=(2, 2), padding="same", activation="swish")(input1)
        norm1 = keras.layers.BatchNormalization()(conv1)
        pool1 = keras.layers.MaxPooling2D(pool_size=(3, 3), strides=(2, 2))(norm1)
        conv2 = keras.layers.Conv2D(filters=32, kernel_size=(3, 3), strides=(2, 2), padding="same", activation="swish")(pool1)
        norm2 = keras.layers.BatchNormalization()(conv2)
        conv3 = keras.layers.Conv2D(filters=32, kernel_size=(3, 3), strides=(1, 1), padding="same", activation="swish")(norm2)
        norm3 = keras.layers.BatchNormalization()(conv3)
        add1 = keras.layers.Add()([norm2, norm3])
        conv4 = keras.layers.Conv2D(filters=64, kernel_size=(3, 3), strides=(2, 2), padding="same", activation="swish")(add1)
        norm4 = keras.layers.BatchNormalization()(conv4)
        conv5 = keras.layers.Conv2D(filters=64, kernel_size=(3, 3), strides=(1, 1), padding="same", activation="swish")(norm4)
        norm5 = keras.layers.BatchNormalization()(conv5)
        add2 = keras.layers.Add()([norm4, norm5])
        conv6 = keras.layers.Conv2D(filters=128, kernel_size=(3, 3), strides=(2, 2), padding="same", activation="swish")(add2)
        norm6 = keras.layers.BatchNormalization()(conv6)
        conv7 = keras.layers.Conv2D(filters=128, kernel_size=(3, 3), strides=(1, 1), padding="same", activation="swish")(norm6)
        norm7 = keras.layers.BatchNormalization()(conv7)
        add3 = keras.layers.Add()([norm6, norm7])
        conv8 = keras.layers.Conv2D(filters=256, kernel_size=(3, 3), strides=(2, 2), padding="same", activation="swish")(add3)
        norm7 = keras.layers.BatchNormalization()(conv8)
        conv9 = keras.layers.Conv2D(filters=512, kernel_size=(3, 3), strides=(2, 2), padding="same", activation="swish")(norm7)
        norm8 = keras.layers.BatchNormalization()(conv9)
        flat1 = keras.layers.Flatten()(norm8)
        dense1 = keras.layers.Dense(128, activation="swish")(flat1)
        norm9 = keras.layers.BatchNormalization()(dense1)
        dense2 = keras.layers.Dense(64, activation="swish")(norm9)
        norm10 = keras.layers.BatchNormalization()(dense2)
        dense3 = keras.layers.Dense(64, activation="swish")(norm10)
        norm11 = keras.layers.BatchNormalization()(dense3)
        dense4 = keras.layers.Dense(2, activation="tanh")(norm11)
        self.model = keras.models.Model(inputs=input1, outputs=dense4)
        self.model.load_weights(self.weight)

    def image_callback(self, msg):
        # convert opencv Mat type to image msg type
        image_ = self.br.imgmsg_to_cv2(msg, 'bgr8')
        
        # resize the image
        resize_img = cv2.resize(image_, (300, 300), cv2.INTER_LINEAR)

        # corp_image
        crop_img = resize_img[190:300, :]

        #get the x_1, x_2 (0~1, 0~1)
        x_1, x_2 = self.model(np.array([crop_img]).astype(np.float32)).numpy()[0]

        # map 0~1 to 0~300 
        cx_1 = int(300 * x_1)
        cx_2 = int(300 * x_2)

        gap = Error()

        ref = 150
        gap.error_1 = ref - cx_1
        gap.error_2 = ref - cx_2

        debug_image = cv2.circle(resize_img, (cx_1, 190), 10, (255, 0, 0), -1)
        debug_image = cv2.circle(debug_image, (cx_2, 190), 10,(0, 0, 255), -1)
        debug_image = cv2.line(debug_image, (ref, 0), (ref, 300),(0,255,0), 5)

        self.debug_publisher.publish(self.br.cv2_to_imgmsg(debug_image, 'bgr8'))
        self.error_publisher.publish(gap)

def main(args=None):
    rclpy.init(args=args)
    dl_detect_line = DLDetectLine()

    rclpy.spin(dl_detect_line)

    dl_detect_line.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()