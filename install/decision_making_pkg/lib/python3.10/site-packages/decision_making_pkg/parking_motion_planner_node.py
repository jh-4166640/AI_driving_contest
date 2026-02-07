import rclpy
import math
from rclpy.node import Node
from rclpy.qos import QoSProfile
from rclpy.qos import QoSHistoryPolicy
from rclpy.qos import QoSDurabilityPolicy
from rclpy.qos import QoSReliabilityPolicy
from std_msgs.msg import Float32
from std_msgs.msg import String, Bool
from interfaces_pkg.msg import PathPlanningResult, DetectionArray, MotionCommand
from .lib import decision_making_func_lib as DMFL

import time

#---------------Variable Setting---------------
SUB_DETECTION_TOPIC_NAME = "detections"
SUB_PATH_TOPIC_NAME = "path_planning_result"
SUB_TRAFFIC_LIGHT_TOPIC_NAME = "yolov8_traffic_light_info"
SUB_LIDAR_OBSTACLE_TOPIC_NAME = "lidar_obstacle_info"
PUB_TOPIC_NAME = "topic_control_signal"
PUB_TOPIC_MOTION_COMMAND = "moves"
TARGET_SLOPE_TOPIC_NAME = "target_slope"
SUB_CAR_DETECTION_TOPIC_NAME = "yolov8_parking_info"
#----------------------------------------------

# 모션 플랜 발행 주기 (초) - 소수점 필요 (int형은 반영되지 않음)
TIMER = 0.1

class MotionPlanningNode(Node):
    def __init__(self):
        super().__init__('motion_planner_node')

        # 토픽 이름 설정
        self.sub_detection_topic = self.declare_parameter('sub_detection_topic', SUB_DETECTION_TOPIC_NAME).value
        self.sub_path_topic = self.declare_parameter('sub_lane_topic', SUB_PATH_TOPIC_NAME).value
        self.sub_traffic_light_topic = self.declare_parameter('sub_traffic_light_topic', SUB_TRAFFIC_LIGHT_TOPIC_NAME).value
        self.sub_lidar_obstacle_topic = self.declare_parameter('sub_lidar_obstacle_topic', SUB_LIDAR_OBSTACLE_TOPIC_NAME).value
        self.sub_car_detection_topic = self.declare_parameter('sub_car_detection_topic', SUB_CAR_DETECTION_TOPIC_NAME).value
        self.pub_topic = self.declare_parameter('pub_topic', PUB_TOPIC_NAME).value
        self.pub_topic_motion_command = self.declare_parameter('pub_topic_motion_command', PUB_TOPIC_MOTION_COMMAND).value
        self.timer_period = self.declare_parameter('timer', TIMER).value
        # target_slope
        self.target_slope_topic = self.declare_parameter('t arget_slope_topic', TARGET_SLOPE_TOPIC_NAME).value
        # QoS 설정
        self.qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            durability=QoSDurabilityPolicy.VOLATILE,
            depth=1
        )

        # 변수 초기화
        self.detection_data = None
        self.path_data = None
        self.traffic_light_data = None
        self.lidar_data = None

        self.steering_command = 0
        self.left_speed_command = 0
        self.right_speed_command = 0
        

        # 서브스크라이버 설정
        self.detection_sub = self.create_subscription(DetectionArray, self.sub_detection_topic, self.detection_callback, self.qos_profile)
        self.path_sub = self.create_subscription(PathPlanningResult, self.sub_path_topic, self.path_callback, self.qos_profile)
        self.traffic_light_sub = self.create_subscription(String, self.sub_traffic_light_topic, self.traffic_light_callback, self.qos_profile)
        self.lidar_sub = self.create_subscription(Bool, self.sub_lidar_obstacle_topic, self.lidar_callback, self.qos_profile)
        self.parking_car_sub = self.create_subscription(String, self.sub_car_detection_topic, self.parking_car_callback, self.qos_profile)

        # 퍼블리셔 설정
        self.publisher = self.create_publisher(MotionCommand, self.pub_topic, self.qos_profile)
        self.motion_command_publisher = self.create_publisher(String, self.pub_topic_motion_command, self.qos_profile)
        self.target_slope_pub = self.create_publisher(Float32, self.target_slope_topic, self.qos_profile)
        # 타이머 설정
        self.timer = self.create_timer(self.timer_period, self.timer_callback)

        self.Init_motion = False




    def detection_callback(self, msg: DetectionArray):
        self.detection_data = msg

    def path_callback(self, msg: PathPlanningResult):
        self.path_data = list(zip(msg.x_points, msg.y_points))
                
    def traffic_light_callback(self, msg: String):
        self.traffic_light_data = msg

    def lidar_callback(self, msg: Bool):
        self.lidar_data = msg

    def parking_car_callback(self, msg: String):
        self.parking_car_data = msg

    def timer_callback(self):

        if self.Init_motion == False:
            self.get_logger().info(f"Start parking motion initialization")
            msg = String()
            msg.data = 'start'
            print(msg.data)
            self.motion_command_publisher.publish(msg)
            self.Init_motion = True
            self.get_logger().info(f"finished parking motion initialization")

        else:
            pass


        # self.get_logger().info(f"steering: {self.steering_command}, " 
        #                        f"left_speed: {self.left_speed_command}, " 
        #                        f"right_speed: {self.right_speed_command}")

        # # 모션 명령 메시지 생성 및 퍼블리시
        # motion_command_msg = MotionCommand()
        # motion_command_msg.steering = self.steering_command
        # motion_command_msg.left_speed = self.left_speed_command
        # motion_command_msg.right_speed = self.right_speed_command
        # self.publisher.publish(motion_command_msg)

def main(args=None):
    rclpy.init(args=args)
    node = MotionPlanningNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\n\nshutdown\n\n")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
