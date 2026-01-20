import zmq
import json
import time
from datetime import datetime
import threading
import sys

class DataReceiver:
    def __init__(self):
        self.context = zmq.Context()
        self.socket = self.context.socket(zmq.SUB)
        self.socket.connect("tcp://localhost:5555")  # 连接到服务器的发布端口
        self.socket.setsockopt_string(zmq.SUBSCRIBE, "")  # 订阅所有消息
        
        # 解析结果存储
        self.parsed_data = {}
        self.running = True

    def parse_pose_message(self, data):
        """解析姿态数据 (POSE消息)"""
        try:
            parts = data.split(',')
            if len(parts) >= 25 and parts[0] == 'POSE':
                parsed = {
                    'type': 'POSE',
                    'servo_angles': [int(x) for x in parts[1:17]],  # 16个舵机角度
                    'timestamp': int(parts[17]),  # 时间戳
                    'imu_data': {
                        'accel_x': float(parts[18]),
                        'accel_y': float(parts[19]),
                        'accel_z': float(parts[20]),
                        'gyro_x': float(parts[21]),
                        'gyro_y': float(parts[22]),
                        'gyro_z': float(parts[23]),
                        'temperature': float(parts[24])
                    },
                    'motor_data': {
                        'directions': [int(x) for x in parts[25:29]],  # 4个电机方向
                        'speeds': [float(x) for x in parts[29:33]]  # 4个电机转速
                    }
                }
                return parsed
        except Exception as e:
            print(f"解析POSE消息失败: {e}")
        return None

    def parse_heartbeat_message(self, data):
        """解析心跳消息 (HB消息)"""
        if data == 'HB':
            return {'type': 'HEARTBEAT'}
        return None

    def parse_ack_message(self, data):
        """解析确认消息 (ACK消息)"""
        if data.startswith('ACK:'):
            try:
                msg_id = int(data.split(':')[1])
                return {'type': 'ACK', 'msg_id': msg_id}
            except:
                pass
        return None

    def parse_error_message(self, data):
        """解析错误消息 (ERR消息)"""
        if data.startswith('ERR:'):
            try:
                parts = data.split(':', 2)
                error_code = int(parts[1])
                error_msg = parts[2] if len(parts) > 2 else "Unknown error"
                return {'type': 'ERROR', 'error_code': error_code, 'error_msg': error_msg}
            except:
                pass
        return None

    def parse_servo_control_message(self, data):
        """解析舵机控制消息 (SERVO消息)"""
        if data.startswith('SERVO,'):
            try:
                parts = data.split(',')
                servo_id = int(parts[1])
                angle = int(parts[2])
                duration = int(parts[3])
                return {'type': 'SERVO_CONTROL', 'servo_id': servo_id, 'angle': angle, 'duration': duration}
            except:
                pass
        return None

    def parse_motor_control_message(self, data):
        """解析电机控制消息 (MOTOR消息)"""
        if data.startswith('MOTOR,'):
            try:
                parts = data.split(',')
                motor_id = int(parts[1])
                signed_speed_rpm = float(parts[2])
                return {'type': 'MOTOR_CONTROL', 'motor_id': motor_id, 'signed_speed_rpm': signed_speed_rpm}
            except:
                pass
        return None

    def parse_message(self, data):
        """解析不同类型的消息"""
        parsers = [
            self.parse_pose_message,
            self.parse_heartbeat_message,
            self.parse_ack_message,
            self.parse_error_message,
            self.parse_servo_control_message,
            self.parse_motor_control_message
        ]
        
        for parser in parsers:
            result = parser(data)
            if result:
                return result
        
        # 如果没有匹配任何预定义格式，返回原始数据
        return {'type': 'UNKNOWN', 'raw_data': data}

    def display_parsed_data(self, parsed_data):
        """显示解析后的数据"""
        print("\n" + "="*60)
        print(f"时间: {datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]}")
        print(f"消息类型: {parsed_data['type']}")
        
        if parsed_data['type'] == 'POSE':
            print("姿态数据:")
            print(f"  舵机角度: {parsed_data['servo_angles']}")
            print(f"  时间戳: {parsed_data['timestamp']}")
            print(f"  IMU数据: 加速度({parsed_data['imu_data']['accel_x']:.2f}, {parsed_data['imu_data']['accel_y']:.2f}, {parsed_data['imu_data']['accel_z']:.2f}), "
                  f"陀螺仪({parsed_data['imu_data']['gyro_x']:.2f}, {parsed_data['imu_data']['gyro_y']:.2f}, {parsed_data['imu_data']['gyro_z']:.2f}), "
                  f"温度: {parsed_data['imu_data']['temperature']:.2f}°C")
            print(f"  电机数据: 方向{parsed_data['motor_data']['directions']}, 转速{parsed_data['motor_data']['speeds']}")
        elif parsed_data['type'] == 'HEARTBEAT':
            print("  心跳包 - 设备在线")
        elif parsed_data['type'] == 'ACK':
            print(f"  确认消息 - ID: {parsed_data['msg_id']}")
        elif parsed_data['type'] == 'ERROR':
            print(f"  错误消息 - 代码: {parsed_data['error_code']}, 内容: {parsed_data['error_msg']}")
        elif parsed_data['type'] == 'SERVO_CONTROL':
            print(f"  舵机控制 - ID: {parsed_data['servo_id']}, 角度: {parsed_data['angle']}, 持续时间: {parsed_data['duration']}ms")
        elif parsed_data['type'] == 'MOTOR_CONTROL':
            signed_speed = parsed_data['signed_speed_rpm']
            if signed_speed > 0:
                direction_info = f"正转({signed_speed} RPM)"
            elif signed_speed < 0:
                direction_info = f"反转({abs(signed_speed)} RPM)"
            else:
                direction_info = "停止(0.0 RPM)"
            print(f"  电机控制 - ID: {parsed_data['motor_id']}, 状态: {direction_info}")
        else:
            print(f"  原始数据: {parsed_data.get('raw_data', 'N/A')}")
        print("="*60)

    def start_receiving(self):
        """开始接收数据"""
        print("开始接收数据... 按 Ctrl+C 退出")
        try:
            while self.running:
                try:
                    # 非阻塞接收
                    message_json = self.socket.recv_string(flags=zmq.NOBLOCK)
                    message = json.loads(message_json)
                    
                    if message['type'] == 'received_from_serial':
                        raw_data = message['data']
                        print(f"[{message['timestamp']}] 接收到原始数据: {raw_data}")
                        
                        # 解析数据
                        parsed_data = self.parse_message(raw_data)
                        if parsed_data:
                            self.display_parsed_data(parsed_data)
                
                except zmq.Again:
                    # 没有新消息，短暂休眠
                    time.sleep(0.01)
                except KeyboardInterrupt:
                    print("\n用户中断，停止接收...")
                    break
                    
        except Exception as e:
            print(f"接收数据时出错: {e}")
        finally:
            self.stop()

    def stop(self):
        """停止接收"""
        self.running = False
        self.socket.close()
        self.context.term()


if __name__ == "__main__":
    receiver = DataReceiver()
    receiver.start_receiving()