import cv2
import random
import numpy as np
from typing import Tuple
import sys, os

import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from rclpy.qos import QoSHistoryPolicy
from rclpy.qos import QoSDurabilityPolicy
from rclpy.qos import QoSReliabilityPolicy

from message_filters import ApproximateTimeSynchronizer, Subscriber
from cv_bridge import CvBridgeQoSDurCarDetectorabilityPolicy

from sensor_msgs.msg import Image
from interfaces_pkg.msg import DetectionArray, BoundingBox2D, Detection
from std_msgs.msg import String

from .lib import camera_perception_func_lib as CPFL

# ---------------Variable Setting---------------
# Subscribe할 토픽 이름
SUB_DETECTION_TOPIC_NAME = "detections"
SUB_IMAGE_TOPIC_NAME = "image_raw"

# Publish할 토픽 이름
PUB_TOPIC_NAME = "yolov8_car_info"

# ----------------------------------------------


class CarDetector(Node):
    def __init__(self):
        super().__init__('traffic_light_detector_node')

        self.sub_detection_topic = self.declare_parameter('sub_detection_topic', SUB_DETECTION_TOPIC_NAME).value
        self.sub_image_topic = self.declare_parameter('sub_image_topic', SUB_IMAGE_TOPIC_NAME).value
        self.pub_topic = self.declare_parameter('pub_topic', PUB_TOPIC_NAME).value

        self.cv_bridge = CvBridge()

        self.qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            durability=QoSDurabilityPolicy.VOLATILE,
            depth=1
        )

        self.detection_sub = Subscriber(self, DetectionArray, self.sub_detection_topic, qos_profile=self.qos_profile)
        self.image_sub = Subscriber(self, Image, self.sub_image_topic, qos_profile=self.qos_profile)
        self.ts = ApproximateTimeSynchronizer([self.detection_sub, self.image_sub], queue_size=1, slop=0.5)
        self.ts.registerCallback(self.sync_callback)

        self.publisher = self.create_publisher(String, self.pub_topic, self.qos_profile)

        self.last_change_time = 0  # 마지막으로 Change를 보낸 시점
        self.cooldown_duration = 1.5  # 쿨타임 (초 단위, 예: 5초 동안은 Change를 다시 안 보냄)

    
    def sync_callback(self, detection_msg: DetectionArray, image_msg: Image):
        cv_image = self.cv_bridge.imgmsg_to_cv2(image_msg)

        center_x_point = 340 # 차량을 인식할 화면상의 x 좌표
        center_y_point = 200 # 차량을 인식할 화면상의 ㅛ 좌표
        center_x_offset = 100 # 인식될 차량의 x 좌표 +-offset
        center_y_offset = 50 # 인식될 차량의 y 좌표 +-offset

        threshold_w_size = 115 # 인식될 차량의 최소 width 사이즈
        threshold_h_size = 115

        current_time = time.time() # 현재 시간 가져오기

        car_detected = False
        for detection in detection_msg.detections:
            if detection.class_name == 'car':

                bbox= detection.bbox
                center_x = bbox.center.position.x
                center_y = bbox.center.position.y
                width = bbox.size.x
                height = bbox.size.y
                
                # Publish traffic light color as string
                car_info = String()
                car_info.data = f"car: ({round(center_x)},{round(center_y)}), ({round(width)},{round(height)})    (cx,cy),(w,h)"

                lane_command = String()
                lane_command.data='None'
                if (width > threshold_w_size and height > threshold_h_size and 
                center_x < center_x_point + center_x_offset and 
                center_x > center_x_point - center_x_offset and 
                center_y < center_y_point + center_y_offset and 
                center_y > center_y_point - center_y_offset):
                    if (current_time - self.last_change_time) > self.cooldown_duration:
                        lane_command.data = 'Change'
                        self.last_change_time = current_time

                print(car_info.data) 
                print(lane_command.data)
                self.publisher.publish(lane_command)
                car_detected = True
                

        if not car_detected:
            # Publish 'None' if no traffic light is detected
            lane_command = String()
            car_info = String()
            car_info.data = 'None'
            lane_command.data = 'None'
            print(f'car_detected: {car_info.data}')
            self.publisher.publish(lane_command)

        print()

def main(args=None):
    rclpy.init(args=args)
    
    node = CarDetector()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\n\nshutdown\n\n")
    finally:
        node.destroy_node()
        cv2.destroyAllWindows()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
