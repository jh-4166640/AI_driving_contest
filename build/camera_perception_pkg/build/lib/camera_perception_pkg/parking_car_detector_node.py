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
from cv_bridge import CvBridge


from sensor_msgs.msg import Image
from interfaces_pkg.msg import DetectionArray, BoundingBox2D, Detection
from std_msgs.msg import String

from .lib import camera_perception_func_lib as CPFL

# ---------------Variable Setting---------------
# Subscribe할 토픽 이름
SUB_DETECTION_TOPIC_NAME = "detections"
SUB_IMAGE_TOPIC_NAME = "image_raw"
SUB_MOTIONS_TOPIC_NAME = "moves"

# Publish할 토픽 이름
PUB_TOPIC_NAME = "yolov8_parking_info"
PUB_TOPIC_PARKING_LOT_INFO = "parking_lot_info"

# ----------------------------------------------


class CarDetector(Node):
    def __init__(self):
        super().__init__('traffic_light_detector_node')

        self.sub_detection_topic = self.declare_parameter('sub_detection_topic', SUB_DETECTION_TOPIC_NAME).value
        self.sub_image_topic = self.declare_parameter('sub_image_topic', SUB_IMAGE_TOPIC_NAME).value
        self.sub_motions_topic = self.declare_parameter('sub_moves_topic', SUB_MOTIONS_TOPIC_NAME).value
        self.pub_topic = self.declare_parameter('pub_topic', PUB_TOPIC_NAME).value
        self.pub_topic_parking_lot_info = self.declare_parameter('pub_parking_lot_info_topic', PUB_TOPIC_PARKING_LOT_INFO).value

        self.cv_bridge = CvBridge()

        self.qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            durability=QoSDurabilityPolicy.VOLATILE,
            depth=1
        )

        # motion_sub에 별도의 콜백 등록
        self.motion_sub = self.create_subscription(
            String, self.sub_motions_topic, self.motion_callback, self.qos_profile)
    

        self.detection_sub = Subscriber(self, DetectionArray, self.sub_detection_topic, qos_profile=self.qos_profile)
        self.image_sub = Subscriber(self, Image, self.sub_image_topic, qos_profile=self.qos_profile)
        self.ts = ApproximateTimeSynchronizer([self.detection_sub, self.image_sub], queue_size=1, slop=0.5)
        self.ts.registerCallback(self.sync_callback)

        self.publisher = self.create_publisher(String, self.pub_topic, self.qos_profile)
        self.publisher_parking_lot_info = self.create_publisher(String, self.pub_topic_parking_lot_info, self.qos_profile)
        
        self.init_motion = False
        self.finded_parking = False

        print("I'm ready to detect parking car after receiving start signal")

    def motion_callback(self, msg):
        # start 신호가 오면 플래그를 True로 변경
        if msg.data == 'start':
            if not self.init_motion:
                self.get_logger().info("Start parking car detector")
            self.init_motion = True
    
    def sync_callback(self, detection_msg: DetectionArray, image_msg: Image):
        if not self.init_motion:
            return
        cv_image = self.cv_bridge.imgmsg_to_cv2(image_msg)

        car_detected = False
        parking_detected = False
        center_x = 0
        c_cx = 0
        center_y = 0
        c_cy = 0
    
        for detection in detection_msg.detections:
            if detection.class_name == 'car':
                c_bbox = detection.bbox
                c_cx = c_bbox.center.position.x
                c_cy = c_bbox.center.position.y
                c_w = c_bbox.size.x
                c_h = c_bbox.size.y
                car_info = String()
                car_info.data = f"car: ({round(c_cx)},{round(c_cy)}), ({round(c_w)},{round(c_h)})    (cx,cy),(w,h)" 
                print(car_info.data)
                car_detected = True

            if detection.class_name == 'parking_lot':
                parking_command = String()
                bbox= detection.bbox
                center_x = bbox.center.position.x
                center_y = bbox.center.position.y
                width = bbox.size.x
                height = bbox.size.y
                info = String()
                info.data = f"parking_lot: ({round(center_x)},{round(center_y)})    (cx,cy),(w,h)"
                parking_lot_info = String()
                parking_lot_info.data = f"cx{round(center_x)} cy{round(center_y)}"
                self.publisher_parking_lot_info.publish(parking_lot_info)
                parking_detected = True
                print(info.data)

        if parking_detected == False:
            parking_lot_info = String()
            parking_lot_info.data = "None"
            self.publisher_parking_lot_info.publish(parking_lot_info)

        if (parking_detected and not self.finded_parking) or (car_detected and not self.finded_parking) or (car_detected and parking_detected and not self.finded_parking):
            parking_lot_info = String()
            if parking_detected == True:
                if( center_x > 230 and center_x < 470):
                    parking_lot_info.data = 'R'
                else :
                    parking_lot_info.data = 'L'
            else:
                if c_cx >= 220 and c_cx <=480 and c_w >= 180:
                    parking_lot_info.data = 'L'
                else:
                    parking_lot_info.data = 'R'
            
                
            self.publisher.publish(parking_lot_info)
            self.finded_parking = True
            print(f'parking_lot_info: {parking_lot_info.data}')

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
