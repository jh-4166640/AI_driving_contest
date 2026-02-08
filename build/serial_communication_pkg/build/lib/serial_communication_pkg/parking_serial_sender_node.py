import time
import serial
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from rclpy.qos import QoSHistoryPolicy
from rclpy.qos import QoSDurabilityPolicy
from rclpy.qos import QoSReliabilityPolicy
from std_msgs.msg import String, Bool
from interfaces_pkg.msg import MotionCommand
from .lib import protocol_convert_func_lib as PCFL

#---------------Variable Setting---------------
# Subscribe할 토픽 이름
SUB_TOPIC_NAME = "topic_control_signal"
PUB_TOPIC_NAME = "parking_ultrasonic_data"

# 아두이노 장치 이름 (ls /dev/ttyA* 명령을 터미널 창에 입력하여 확인)
PORT='/dev/ttyACM0'
#----------------------------------------------

ser = serial.Serial(PORT, 115200, timeout=1)
time.sleep(1)
BAUDRATE = 115200

class SerialSenderNode(Node):
    def __init__(self, sub_topic=SUB_TOPIC_NAME):
        super().__init__('serial_sender_node')

        self.pub_topic = self.declare_parameter('pub_topic', PUB_TOPIC_NAME).value # [ultrasonic data publish topic]
        
        self.declare_parameter('sub_topic', sub_topic)
        
        self.sub_topic = self.get_parameter('sub_topic').get_parameter_value().string_value
        
        qos_profile = QoSProfile(reliability=QoSReliabilityPolicy.RELIABLE, 
                                history=QoSHistoryPolicy.KEEP_LAST, 
                                durability=QoSDurabilityPolicy.VOLATILE, 
                                depth=1)
        
        self.subscription = self.create_subscription(MotionCommand, self.sub_topic, self.data_callback, qos_profile)
        self.ultrasonic_pub = self.create_publisher(String, self.pub_topic, qos_profile)
        # 3. Timer 설정 (Receiver 기능 - 50Hz 정도로 수신 확인)
            # 아두이노가 COMMAND_INTERVAL(50ms)마다 보내므로 0.02s 주기가 적당합니다.
        self.timer = self.create_timer(0.02, self.receive_sensor_data)

    def data_callback(self, msg):
        steering = msg.steering
        left_speed = msg.left_speed
        right_speed = msg.right_speed

        serial_msg =  PCFL.convert_serial_message(steering, left_speed, right_speed)
        ser.write(serial_msg.encode())


    def receive_sensor_data(self):
        """아두이노로부터 초음파 데이터를 읽어와 파싱 (Receiver)"""
        if ser.in_waiting > 0:
            try:
                line = ser.readline().decode('utf-8').strip()
                if line and line.startswith("LR"): # 초음파 데이터 포맷인지 확인
                    msg = String()
                    msg.data = line
                    self.ultrasonic_pub.publish(msg)
                    
                    parts = line.split()
                    dist = {}
                    for part in parts:
                        key = part[:2]
                        val = int(part[2:])
                        dist[key] = val
                    
                    # 수신 데이터 로그 출력
                    self.get_logger().info(
                        f"Distances -> LR: {dist.get('LR')} RR: {dist.get('RR')} LS: {dist.get('LS')} RS: {dist.get('RS')}"
                    )
            except Exception as e:
                # 데이터가 잘리거나 깨진 경우 무시
                pass
    
    def stop_robot(self):
        """종료 시 로봇 정지 명령 전송"""
        message = PCFL.convert_serial_message(0, 0, 0)
        self.ser.write(message.encode())
        self.ser.close()

def main(args=None):
    rclpy.init(args=args)
    node = SerialSenderNode()
    try:
        rclpy.spin(node)
        
    except KeyboardInterrupt:
        print("\n\nshutdown\n\n")
        steering = 0
        left_speed = 0
        right_speed = 0
        message = PCFL.convert_serial_message(steering, left_speed, right_speed)
        ser.write(message.encode())
        pass
        
    finally:
        ser.close()
        print('closed')
        
    node.destroy_node()
    rclpy.shutdown()
  
if __name__ == '__main__':
    main()
