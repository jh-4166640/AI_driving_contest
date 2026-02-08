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
SUB_PUB_TOPIC_PARKING_LOT_INFO = "parking_lot_info"
SUB_ULTRASONIC_TOPIC_NAME = "parking_ultrasonic_data"
#----------------------------------------------

PARKING_SPEED = -90
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
        self.sub_parking_lot_info_topic = self.declare_parameter('sub_parking_lot_info_topic', SUB_PUB_TOPIC_PARKING_LOT_INFO).value
        self.sub_ultrasonic_topic = self.declare_parameter('sub_ultrasonic_topic', SUB_ULTRASONIC_TOPIC_NAME).value
        self.pub_topic = self.declare_parameter('pub_topic', PUB_TOPIC_NAME).value
        self.pub_topic_motion_command = self.declare_parameter('pub_topic_motion_command', PUB_TOPIC_MOTION_COMMAND).value
        self.timer_period = self.declare_parameter('timer', TIMER).value
        # target_slope
        self.target_slope_topic = self.declare_parameter('target_slope_topic', TARGET_SLOPE_TOPIC_NAME).value
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
        self.parking_lot_info_sub = self.create_subscription(String, self.sub_parking_lot_info_topic, self.parking_lot_info_callback, self.qos_profile)
        self.ultrasonic_sub = self.create_subscription(String, self.sub_ultrasonic_topic, self.ultrasonic_callback, self.qos_profile)
        # 퍼블리셔 설정
        self.publisher = self.create_publisher(MotionCommand, self.pub_topic, self.qos_profile)
        self.motion_command_publisher = self.create_publisher(String, self.pub_topic_motion_command, self.qos_profile)
        self.target_slope_pub = self.create_publisher(Float32, self.target_slope_topic, self.qos_profile)
        # 타이머 설정
        self.timer = self.create_timer(self.timer_period, self.timer_callback)
        self.Init_motion = False
        self.parking_located = False
        self.parking_car_data = None
        self.parking_lot_info_data = None
        self.current_cx = None
        self.current_cy = None
        self.parking_completed = False

        self.lr = 999
        self.rr = 999
        self.ls = 999
        self.rs = 999

        self.backP = False
        time.sleep(3.0)



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

    def Motion_function(self,steer, speed, t):
        self.steering_command = steer  # 예시 조향 값 (7이 최대 조향)
        self.left_speed_command = speed  # 예시 속도 값 (255가 최대 속도)
        self.right_speed_command = speed  # 예시 속도 값 (255가 최대 속도)
        # 차선 변경 후 플래그 초기화
        self.get_logger().info(f"steering: {self.steering_command}, " 
                            f"left_speed: {self.left_speed_command}, " 
                            f"right_speed: {self.right_speed_command}")

        # 모션 명령 메시지 생성 및 퍼블리시
        motion_command_msg = MotionCommand()
        motion_command_msg.steering = self.steering_command
        motion_command_msg.left_speed = self.left_speed_command
        motion_command_msg.right_speed = self.right_speed_command
        self.publisher.publish(motion_command_msg)
        time.sleep(t)  # 차선 변경 동작을 위한 대기 시간 (필요에 따라 조정)

    def ultrasonic_callback(self, msg):
        """
        'LR10 RR20 LS30 RS40' 문자열을 개별 변수에 저장
        """
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

            # 값 확인용 로그 (필요 없으면 삭제 가능)
            # self.get_logger().info(f"Updated -> LR:{self.lr}, RR:{self.rr}, LS:{self.ls}, RS:{self.rs}")

        except Exception as e:
            # 데이터가 불완전하게 들어올 경우를 대비한 예외 처리
            pass
    
    def parking_lot_info_callback(self, msg: String):

        if msg.data == "None":
            # self.get_logger().info("No parking lot detected")
            self.current_cx = None
            self.current_cy = None
            return
        # msg.data 예시: "cx320 cy240"
        try:
            # 공백으로 나누기 -> ["cx320", "cy240"]
            parts = msg.data.split()
            
            # 각 문자열에서 숫자 부분만 슬라이싱 후 정수로 변환
            # "cx320"[2:] -> "320" -> 320
            cx = int(parts[0][2:])
            cy = int(parts[1][2:])
            
            # 이제 숫자 값으로 활용 가능
            # self.get_logger().info(f"Parsed Coordinates: X={cx}, Y={cy}")
            
            # 기존 data 변수에는 판정 로직을 위해 전체 메시지 저장 가능
            self.parking_lot_info_data = msg 
            # 혹은 클래스 변수에 좌표 저장
            self.current_cx = cx
            self.current_cy = cy

        except Exception as e:
            self.get_logger().error(f"Parsing error: {e}")



    def timer_callback(self):

        if self.Init_motion == False:

            self.get_logger().info(f"Start parking motion initialization")

            # self.Motion_function(7,140,1.8)
            # self.Motion_function(-7,140,7.5)
            # self.Motion_function(7,0,1.0)
            # self.Motion_function(7,-135,3.2)
            # self.Motion_function(-7,0,1.0)
            # self.Motion_function(-7,135,2.4)
            # self.Motion_function(0,0,1.0)
            # self.Motion_function(0,-135,0.8)
            # self.Motion_function(-7,-135,2.3)
            # self.Motion_function(5,0,1.0)
            # self.Motion_function(7,135,1.2)
            # self.Motion_function(-4,0,1.0)
            # self.Motion_function(-4,-135,1.2)
            # self.Motion_function(0,0,1.0)

            
            self.Motion_function(7,140,1.8)
            self.Motion_function(-7,140,6.2)
            self.Motion_function(7,0,1.0)
            self.Motion_function(7,-135,2.65)
            self.Motion_function(0,-135,1.0)
            self.Motion_function(0,0,1.0)
            

            # self.Motion_function(7,135,1.4)
            # self.Motion_function(-5,135,2.0)
            # self.Motion_function(0,0,1.0)
            # self.Motion_function(0,-135,1.2)
            # self.Motion_function(0,0,1.0)



            msg = String()
            msg.data = 'start'
            print(msg.data)
            for i in range(100):
                self.motion_command_publisher.publish(msg)
            self.Init_motion = True
            self.get_logger().info(f"finished parking motion initialization")

            # finished init
            self.Motion_function(0,0,1.51)
            
        elif self.parking_located == False:
            if self.parking_car_data is not None and (self.parking_car_data.data == "L" or self.parking_car_data.data == "R"):
                print(f"break Find!")
                print(f"parking_car_data: {self.parking_car_data.data}")
                self.parking_located = True
        
            # elif self.parking_located == False and self.parking_car_data is None:
            #     self.get_logger().info("No parking car data received yet")
            #     self.Motion_function(0,135,1.5)
            #     self.Motion_function(0,0,1.0)
            #     self.Motion_function(0,-135,1.5)
            #     self.Motion_function(0,0,1.0)

        elif self.parking_located == True and self.parking_completed == False:
            if self.parking_car_data.data == "R":
                if self.backP == False and (self.rr < 100 or self.lr < 100 or self.ls < 100 or self.rs < 100):
                    self.get_logger().info("Parking process start")
                    self.backP = True

                elif self.backP == False:
                    self.steering_command = 0  # 예시 조향 값 (7이 최대 조향)
                    self.left_speed_command = -135  # 예시 속도 값 (255가 최대 속도)
                    self.right_speed_command = -135  # 예시 속도 값 (255가 최대 속도)
            
                    motion_command_msg = MotionCommand()
                    motion_command_msg.steering = self.steering_command
                    motion_command_msg.left_speed = self.left_speed_command
                    motion_command_msg.right_speed = self.right_speed_command
                    self.publisher.publish(motion_command_msg)
                
                elif self.backP == True:
                    if self.rr > 100 and self.lr > 100 and self.ls < 80 and self.rs <80:
                        self.get_logger().info("Parking process completed")
                        self.get_logger().info(f"Final Distances -> LR: {self.lr} RR: {self.rr} LS: {self.ls} RS: {self.rs}")
                        self.parking_completed = True
                        self.steering_command = 0  # 예시 조향 값 (7이
                        self.left_speed_command = 0  # 예시 속도 값 (255가 최대 속도)
                        self.right_speed_command = 0  # 예시 속도 값 (255가 최대
                        
                        motion_command_msg = MotionCommand()
                        motion_command_msg.steering = self.steering_command
                        motion_command_msg.left_speed = self.left_speed_command
                        motion_command_msg.right_speed = self.right_speed_command
                        self.publisher.publish(motion_command_msg)
                        

                    elif self.lr > 60 and self.rr < 53 and self.ls < 80 and self.rs <80:
                        self.get_logger().info("case 1")
                        self.steering_command = -4  # 예시 조향 값 (7이 최대 조향)
                        self.left_speed_command = PARKING_SPEED  # 예시 속도 값 (255가 최대 속도)
                        self.right_speed_command = PARKING_SPEED  # 예시 속도 값 (255가 최대 속도)
                        
                        motion_command_msg = MotionCommand()
                        motion_command_msg.steering = self.steering_command
                        motion_command_msg.left_speed = self.left_speed_command
                        motion_command_msg.right_speed = self.right_speed_command
                        self.publisher.publish(motion_command_msg)
                        
                    elif self.rr > 60 and self.lr < 53 and self.ls < 80 and self.rs <80:
                        self.get_logger().info("case 2")
                        self.steering_command = 4  # 예시 조향 값 (7이 최대 조향)
                        self.left_speed_command = PARKING_SPEED # 예시 속도 값 (255가 최대 속도)
                        self.right_speed_command = PARKING_SPEED  # 예시 속도 값 (255가 최대 속도)
                        
                        motion_command_msg = MotionCommand()
                        motion_command_msg.steering = self.steering_command
                        motion_command_msg.left_speed = self.left_speed_command
                        motion_command_msg.right_speed = self.right_speed_command
                        self.publisher.publish(motion_command_msg)


                    elif self.lr > 60 and self.rr < 60:
                        self.get_logger().info("case 3")
                        self.steering_command = -4  # 예시 조향 값 (7이 최대 조향)
                        self.left_speed_command = PARKING_SPEED  # 예시 속도 값 (255가 최대 속도)
                        self.right_speed_command = PARKING_SPEED  # 예시 속도 값 (255가 최대 속도)
                        
                        motion_command_msg = MotionCommand()
                        motion_command_msg.steering = self.steering_command
                        motion_command_msg.left_speed = self.left_speed_command
                        motion_command_msg.right_speed = self.right_speed_command
                        self.publisher.publish(motion_command_msg)

                    elif self.rr > 60 and self.lr < 60:
                        self.get_logger().info("case 4")
                        self.steering_command = 4  # 예시 조향 값 (7이 최대 조향)
                        self.left_speed_command = PARKING_SPEED  # 예시 속도 값 (255가 최대 속도)
                        self.right_speed_command = PARKING_SPEED  # 예시 속도 값 (255가 최대 속도)
                        
                        motion_command_msg = MotionCommand()
                        motion_command_msg.steering = self.steering_command
                        motion_command_msg.left_speed = self.left_speed_command
                        motion_command_msg.right_speed = self.right_speed_command
                        self.publisher.publish(motion_command_msg)

                    elif self.rr < 60 and self.lr < 60:
                        self.get_logger().info("case 5")
                        self.steering_command = 0  # 예시 조향 값 (7이 최대 조향)
                        self.left_speed_command = PARKING_SPEED  # 예시 속도 값 (255가 최대 속도)
                        self.right_speed_command = PARKING_SPEED  # 예시 속도 값 (255가 최대 속도)
                        
                        motion_command_msg = MotionCommand()
                        motion_command_msg.steering = self.steering_command
                        motion_command_msg.left_speed = self.left_speed_command
                        motion_command_msg.right_speed = self.right_speed_command
                        self.publisher.publish(motion_command_msg)

                    else :
                        self.get_logger().info("case 6")
                        self.steering_command = 0  # 예시 조향 값 (7이 최대 조향)
                        self.left_speed_command = PARKING_SPEED  # 예시 속도 값 (255가 최대 속도)
                        self.right_speed_command = PARKING_SPEED  # 예시 속도 값 (255가 최대 속도)
                        
                        motion_command_msg = MotionCommand()
                        motion_command_msg.steering = self.steering_command
                        motion_command_msg.left_speed = self.left_speed_command
                        motion_command_msg.right_speed = self.right_speed_command
                        self.publisher.publish(motion_command_msg)
                

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
