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
SUB_MOTIONS_TOPIC_NAME = "moves"

# Publish할 토픽 이름
PUB_TOPIC_NAME = "parking_lot_info"
SUB_ULTRASONIC_TOPIC_NAME = "parking_ultrasonic_data"

# ----------------------------------------------


class CarDetector(Node):
    def __init__(self):
        super().__init__('traffic_light_detector_node')

        self.sub_motions_topic = self.declare_parameter('sub_moves_topic', SUB_MOTIONS_TOPIC_NAME).value
        self.pub_topic = self.declare_parameter('pub_topic', PUB_TOPIC_NAME).value
        self.sub_ultrasonic_topic = self.declare_parameter('sub_ultrasonic_topic', SUB_ULTRASONIC_TOPIC_NAME).value
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
    

        self.ultrasonic_sub = self.create_subscription(String, self.sub_ultrasonic_topic, self.ultrasonic_callback, self.qos_profile)
        self.publisher = self.create_publisher(String, self.pub_topic, self.qos_profile)
        
        self.init_motion = False
        self.finded_parking = False

        print("I'm ready to detect parking car after receiving start signal")

    def motion_callback(self, msg):
        # start 신호가 오면 플래그를 True로 변경
        if msg.data == 'start':
            if not self.init_motion:
                self.get_logger().info("Start parking car detector")
            self.init_motion = True

    def ultrasonic_callback(self, msg):
        if self.init_motion == False:
            return
        else:
            raw_data = msg.data
            try:
                # 공백 기준 분리 ['LR10', 'RR20', 'LS30', 'RS40']
                parts = raw_data.split()

                for part in parts:
                    prefix = part[:2]   # 앞 2글자 (LR, RR 등)
                    value = int(part[2:]) # 나머지 숫자

                    # 접두사에 따라 해당 변수에 즉시 저장
                    if prefix == "LR":
                        self.lr = value
                    elif prefix == "RR":
                        self.rr = value
                    elif prefix == "LS":
                        self.ls = value
                    elif prefix == "RS":
                        self.rs = value
                    elif prefix == "BC":
                        self.bc = value
                    elif prefix == "BL":
                        self.bl = value

                # 값 확인용 로그 (필요 없으면 삭제 가능)
                # self.get_logger().info(f"Updated -> LR:{self.lr}, RR:{self.rr}, LS:{self.ls}, RS:{self.rs}")

            except Exception as e:
                # 데이터가 불완전하게 들어올 경우를 대비한 예외 처리
                pass
            if self.finded_parking == False:
                if self.bl > 100 and self.bc < 100:
                    parking_lot_info = String()
                    parking_lot_info.data = "L"
                    self.publisher.publish(parking_lot_info)

                else :
                    parking_lot_info = String()
                    parking_lot_info.data = "R"
                    self.publisher.publish(parking_lot_info)
                
                self.get_logger().info(f"Detected parking lot on the RIGHT side! BC: {self.bc}, BL: {self.bl}")
                self.finded_parking = True

            

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
