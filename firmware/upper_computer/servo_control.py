# import serial
# import time

# # 打开串口（根据实际情况选择端口）
# ser = serial.Serial('/dev/ttyTHS1', 115200, timeout=1)

# def send_servo_command(servo_id, angle, duration=500):
#     """
#     发送舵机控制命令

#     Args:
#         servo_id: 舵机ID (0-15)
#         angle: 目标角度 (0-180度)
#         duration: 执行时间 (毫秒)
#     """
#     command = f"SERVO,{servo_id},{angle},{duration}\n"
#     ser.write(command.encode())
#     print(f"发送舵机控制命令: {command.strip()}")

# def parse_pose_data(data):
#     """
#     解析姿态数据

#     Args:
#         data: 接收到的数据字符串
#     """
#     if data.startswith("POSE,"):
#         try:
#             parts = data.split(',')
#             if len(parts) >= 24:  # POSE,16个舵机角度,时间戳,6个IMU值,温度
#                 servo_angles = [int(parts[i]) for i in range(1, 17)]  # 舵机角度
#                 timestamp = int(parts[17])  # 时间戳
#                 accel_x = float(parts[18])  # 加速度X
#                 accel_y = float(parts[19])  # 加速度Y
#                 accel_z = float(parts[20])  # 加速度Z
#                 gyro_x = float(parts[21])   # 陀螺仪X
#                 gyro_y = float(parts[22])   # 陀螺仪Y
#                 gyro_z = float(parts[23])   # 陀螺仪Z
#                 temperature = float(parts[24]) if len(parts) > 24 else 0.0  # 温度

#                 print(f"解析姿态数据:")
#                 print(f"  舵机角度: {servo_angles}")
#                 print(f"  IMU数据: Accel({accel_x:.2f}, {accel_y:.2f}, {accel_z:.2f}), "
#                       f"Gyro({gyro_x:.2f}, {gyro_y:.2f}, {gyro_z:.2f}), Temp: {temperature:.2f}°C")
#                 return {
#                     'servo_angles': servo_angles,
#                     'timestamp': timestamp,
#                     'accel': (accel_x, accel_y, accel_z),
#                     'gyro': (gyro_x, gyro_y, gyro_z),
#                     'temperature': temperature
#                 }
#         except Exception as e:
#             print(f"解析姿态数据失败: {e}")
#     return None

# def receive_data():
#     """接收数据"""
#     if ser.in_waiting > 0:
#         data = ser.read(ser.in_waiting).decode('utf-8', errors='ignore')
#         lines = data.strip().split('\n')
#         for line in lines:
#             line = line.strip()
#             if line:
#                 parsed_data = parse_pose_data(line)
#                 if not parsed_data:
#                     print(f"收到其他数据: {line}")

# # 初始化舵机到中间位置
# print("初始化舵机到90度...")
# send_servo_command(0,90, 1000)  # 0号舵机转到90度，用时1秒
# # time.sleep(1)

# # send_servo_command(1, 90, 1000)  # 1号舵机转到90度，用时1秒
# # time.sleep(1)

# # 主循环
# # counter = 0
# # while True:
#     # 每隔几秒发送一个舵机控制命令
# #    if counter % 100 == 0:  # 每1秒（100次循环）发送一次舵机命令
#         # 切换舵机角度
# #        servo_id = (counter // 500) % 2  # 在0号和1号舵机之间切换
# #        angle = 0 if (counter // 1000) % 2 == 0 else 180  # 在45度和135度之间切换
# #        send_servo_command(servo_id, angle, 500)

#     # 接收数据
# #    receive_data()

#     # 延迟以达到100Hz频率
# #    time.sleep(0.01)  # 10ms = 100Hz
# #    counter += 1



# import serial
# import time

# # 打开串口（根据实际情况选择端口）
# ser = serial.Serial('/dev/ttyTHS1', 115200, timeout=1)

# def send_servo_command(servo_id, angle, duration=500):
#     """
#     发送舵机控制命令

#     Args:
#         servo_id: 舵机ID (0-15)
#         angle: 目标角度 (0-180度)
#         duration: 执行时间 (毫秒)
#     """
#     command = f"SERVO,{servo_id},{angle},{duration}\n"
#     ser.write(command.encode())
#     print(f"发送舵机控制命令: {command.strip()}")

# def send_motor_command(motor_id, command, speed_rpm):
#     """
#     发送电机控制命令

#     Args:
#         motor_id: 电机ID (0-3)
#         command: 命令 (0:慢刹车, 1:快刹车, 2:反转, 3:正转)
#         speed_rpm: 转速 (RPM)
#     """
#     command_str = f"MOTOR,{motor_id},{command},{speed_rpm}\n"
#     ser.write(command_str.encode())
#     # 显示命令含义
#     cmd_names = ["慢刹车", "快刹车", "反转", "正转"]
#     print(f"发送电机控制命令: {command_str.strip()} ({cmd_names[command]})")

# def parse_pose_data(data):
#     """
#     解析姿态数据n: [Errno 2] could not open port /dev/ttyUSB0: [Errno 2] No such file or directory: '/dev/ttyUSB0'
# hjc@hjc-desktop:~$ 


#     Args:
#         data: 接收到的数据字符串
#     """
#     if data.startswith("POSE,"):
#         try:
#             parts = data.split(',')
#             if len(parts) >= 32:  # POSE,16个舵机角度,时间戳,6个IMU值,温度,4个电机转向,4个电机转速
#                 servo_angles = [int(parts[i]) for i in range(1, 17)]  # 舵机角度
#                 timestamp = int(parts[17])  # 时间戳
#                 accel_x = float(parts[18])  # 加速度X
#                 accel_y = float(parts[19])  # 加速度Y
#                 accel_z = float(parts[20])  # 加速度Z
#                 gyro_x = float(parts[21])   # 陀螺仪X
#                 gyro_y = float(parts[22])   # 陀螺仪Y
#                 gyro_z = float(parts[23])   # 陀螺仪Z
#                 temperature = float(parts[24]) if len(parts) > 24 else 0.0  # 温度

#                 # 电机数据
#                 motor_directions = [int(parts[i]) for i in range(25, 29)]  # 4个电机转向
#                 motor_speeds = [float(parts[i]) for i in range(29, 33)]    # 4个电机转速

#                 print(f"解析姿态数据:")
#                 print(f"  舵机角度: {servo_angles}")
#                 print(f"  IMU数据: Accel({accel_x:.2f}, {accel_y:.2f}, {accel_z:.2f}), "
#                       f"Gyro({gyro_x:.2f}, {gyro_y:.2f}, {gyro_z:.2f}), Temp: {temperature:.2f}°C")
#                 print(f"  电机转向: {motor_directions}")
#                 print(f"  电机转速: {motor_speeds}")
#                 return {
#                     'servo_angles': servo_angles,
#                     'timestamp': timestamp,
#                     'accel': (accel_x, accel_y, accel_z),
#                     'gyro': (gyro_x, gyro_y, gyro_z),
#                     'temperature': temperature,
#                     'motor_directions': motor_directions,
#                     'motor_speeds': motor_speeds
#                 }
#         except Exception as e:
#             print(f"解析姿态数据失败: {e}")
#     return None

# def receive_data():
#     """接收数据"""
#     if ser.in_waiting > 0:
#         data = ser.read(ser.in_waiting).decode('utf-8', errors='ignore')
#         lines = data.strip().split('\n')
#         for line in lines:
#             line = line.strip()
#             if line:
#                 parsed_data = parse_pose_data(line)
#                 if not parsed_data:
#                     print(f"收到其他数据: {line}")

# # 初始化舵机到中间位置
# print("初始化舵机到90度...")
# send_servo_command(0, 90, 1000)  # 0号舵机转到90度，用时1秒

# # 初始化电机控制
# print("初始化电机控制...")

# # 定义电机控制参数
# MOTOR_SPEED = 50.0  # 设定电机转速为50 RPM
# motors = [0, 1, 2, 3]  # 四个电机ID

# # 程序开始时让四个电机快刹车
# print("程序开始：让四个电机快刹车")
# for motor_id in motors:
#     send_motor_command(motor_id, 1, 0.0)  # 命令1=快刹车，转速为0

# # 记录初始时间
# start_time = time.time()
# last_command_change = start_time
# command_sequence = [3, 1, 2, 0]  # 正转、快刹车、反转、慢刹车
# current_command_index = 0  # 当前命令索引

# print("开始主循环，每10秒按顺序改变电机命令...")
# try:
#     while True:
#         current_time = time.time()

#         # 检查是否需要改变电机命令
#         if current_time - last_command_change >= 10:
#             # 获取当前命令
#             current_command = command_sequence[current_command_index]

#             # 根据命令类型显示信息
#             cmd_names = ["慢刹车", "快刹车", "反转", "正转"]
#             print(f"{int(current_time - start_time)}秒: 让四个电机{cmd_names[current_command]}")

#             # 发送新的电机控制命令
#             for motor_id in motors:
#                 if current_command == 0:  # 慢刹车
#                     send_motor_command(motor_id, 0, 0.0)
#                 elif current_command == 1:  # 快刹车
#                     send_motor_command(motor_id, 1, 0.0)
#                 elif current_command == 2:  # 反转
#                     send_motor_command(motor_id, 2, MOTOR_SPEED)
#                 elif current_command == 3:  # 正转
#                     send_motor_command(motor_id, 3, MOTOR_SPEED)

#             # 更新命令索引，循环使用命令序列
#             current_command_index = (current_command_index + 1) % len(command_sequence)
#             last_command_change = current_time

#         # 接收数据
#         receive_data()

#         # 延迟以达到100Hz频率
#         time.sleep(0.01)  # 10ms = 100Hz

# except KeyboardInterrupt:
#     print("\n程序被用户中断")
#     # 程序结束前让电机快刹车
#     print("让所有电机快刹车...")
#     for motor_id in motors:
#         send_motor_command(motor_id, 1, 0.0)  # 命令1=快刹车
#     ser.close()
#     print("串口已关闭")



import serial
import time
import sys

class RobotController:
    def __init__(self, port='/dev/ttyTHS1', baudrate=115200, timeout=1):
        """
        初始化机器人控制器
        
        Args:
            port: 串口端口，可以是 /dev/ttyTHS1, /dev/ttyUSB0, /dev/ttyACM0, COM3 等
            baudrate: 波特率
            timeout: 超时时间
        """
        self.serial_port = None
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        
        # 电机控制参数
        self.MOTOR_SPEED = 50.0
        self.motors = [0, 1, 2, 3]
        
        # 命令序列
        self.command_sequence = [3, 1, 2, 0]  # 正转、快刹车、反转、慢刹车
        self.command_names = ["慢刹车", "快刹车", "反转", "正转"]
        
        # 时间记录
        self.start_time = 0
        self.last_command_change = 0
        self.current_command_index = 0
        
    def connect(self):
        """连接到串口设备"""
        ports_to_try = [
            self.port,
            '/dev/ttyUSB0',
            '/dev/ttyUSB1',
            '/dev/ttyACM0',
            '/dev/ttyACM1',
            'COM3',
            'COM4'
        ]
        
        for port in ports_to_try:
            try:
                print(f"尝试连接端口: {port}")
                self.serial_port = serial.Serial(
                    port=port,
                    baudrate=self.baudrate,
                    timeout=self.timeout
                )
                print(f"成功连接到 {port}")
                return True
            except serial.SerialException as e:
                print(f"无法连接到 {port}: {e}")
        
        print("无法找到可用的串口设备")
        return False
    
    def disconnect(self):
        """断开串口连接"""
        if self.serial_port and self.serial_port.is_open:
            self.serial_port.close()
            print("串口已关闭")
    
    def send_servo_command(self, servo_id, angle, duration=500):
        """
        发送舵机控制命令
        
        Args:
            servo_id: 舵机ID (0-15)
            angle: 目标角度 (0-180度)
            duration: 执行时间 (毫秒)
        """
        if not self.serial_port or not self.serial_port.is_open:
            print("错误: 串口未连接")
            return False
            
        command = f"SERVO,{servo_id},{angle},{duration}\n"
        try:
            self.serial_port.write(command.encode())
            print(f"发送舵机控制命令: {command.strip()}")
            return True
        except Exception as e:
            print(f"发送舵机命令失败: {e}")
            return False
    
    def send_motor_command(self, motor_id, command, speed_rpm):
        """
        发送电机控制命令
        
        Args:
            motor_id: 电机ID (0-3)
            command: 命令 (0:慢刹车, 1:快刹车, 2:反转, 3:正转)
            speed_rpm: 转速 (RPM)
        """
        if not self.serial_port or not self.serial_port.is_open:
            print("错误: 串口未连接")
            return False
            
        command_str = f"MOTOR,{motor_id},{command},{speed_rpm}\n"
        try:
            self.serial_port.write(command_str.encode())
            print(f"发送电机控制命令: {command_str.strip()} ({self.command_names[command]})")
            return True
        except Exception as e:
            print(f"发送电机命令失败: {e}")
            return False
    
    def parse_pose_data(self, data):
        """
        解析姿态数据
        
        Args:
            data: 接收到的数据字符串
        """
        if data.startswith("POSE,"):
            try:
                parts = data.split(',')
                if len(parts) >= 32:
                    # 解析舵机角度
                    servo_angles = [int(parts[i]) for i in range(1, 17)]
                    
                    # 解析时间戳
                    timestamp = int(parts[17])
                    
                    # 解析IMU数据
                    accel_x, accel_y, accel_z = float(parts[18]), float(parts[19]), float(parts[20])
                    gyro_x, gyro_y, gyro_z = float(parts[21]), float(parts[22]), float(parts[23])
                    
                    # 解析温度
                    temperature = float(parts[24]) if len(parts) > 24 else 0.0
                    
                    # 解析电机数据
                    motor_directions = [int(parts[i]) for i in range(25, 29)]
                    motor_speeds = [float(parts[i]) for i in range(29, 33)]
                    
                    print(f"解析姿态数据:")
                    print(f"  时间戳: {timestamp}ms")
                    print(f"  舵机角度: {servo_angles}")
                    print(f"  加速度: ({accel_x:.2f}, {accel_y:.2f}, {accel_z:.2f}) m/s²")
                    print(f"  陀螺仪: ({gyro_x:.2f}, {gyro_y:.2f}, {gyro_z:.2f}) °/s")
                    print(f"  温度: {temperature:.1f}°C")
                    print(f"  电机转向: {motor_directions}")
                    print(f"  电机转速: {motor_speeds}")
                    
                    return {
                        'servo_angles': servo_angles,
                        'timestamp': timestamp,
                        'accel': (accel_x, accel_y, accel_z),
                        'gyro': (gyro_x, gyro_y, gyro_z),
                        'temperature': temperature,
                        'motor_directions': motor_directions,
                        'motor_speeds': motor_speeds
                    }
            except (ValueError, IndexError) as e:
                print(f"解析姿态数据失败: {e}, 原始数据: {data}")
        return None
    
    def receive_data(self):
        """接收数据"""
        if not self.serial_port or not self.serial_port.is_open:
            return None
            
        try:
            if self.serial_port.in_waiting > 0:
                data = self.serial_port.read(self.serial_port.in_waiting).decode('utf-8', errors='ignore')
                lines = data.strip().split('\n')
                
                results = []
                for line in lines:
                    line = line.strip()
                    if line:
                        parsed_data = self.parse_pose_data(line)
                        if parsed_data:
                            results.append(parsed_data)
                        elif line and not line.startswith("POSE,"):
                            print(f"收到其他数据: {line}")
                
                return results
        except Exception as e:
            print(f"接收数据失败: {e}")
        
        return None
    
    def initialize(self):
        """初始化机器人"""
        print("初始化机器人...")
        
        # 初始化舵机
        print("初始化舵机到90度...")
        for servo_id in range(4):  # 只初始化前4个舵机
            self.send_servo_command(servo_id, 90, 1000)
            time.sleep(0.2)
        
        # 初始化电机（全部快刹车）
        print("初始化电机控制（快刹车）...")
        for motor_id in self.motors:
            self.send_motor_command(motor_id, 1, 0.0)
            time.sleep(0.1)
        
        print("初始化完成")
    
    def run_motor_sequence(self):
        """运行电机控制序列"""
        print("开始运行电机控制序列...")
        
        self.start_time = time.time()
        self.last_command_change = self.start_time
        self.current_command_index = 0
        
        try:
            while True:
                current_time = time.time()
                elapsed_time = current_time - self.start_time
                
                # 每10秒改变一次电机命令
                if current_time - self.last_command_change >= 10:
                    current_command = self.command_sequence[self.current_command_index]
                    cmd_name = self.command_names[current_command]
                    
                    print(f"[{int(elapsed_time)}秒] 设置所有电机为: {cmd_name}")
                    
                    # 发送新的电机控制命令
                    for motor_id in self.motors:
                        if current_command in [0, 1]:  # 刹车命令
                            self.send_motor_command(motor_id, current_command, 0.0)
                        else:  # 转动命令
                            self.send_motor_command(motor_id, current_command, self.MOTOR_SPEED)
                        time.sleep(0.05)
                    
                    # 更新命令索引
                    self.current_command_index = (self.current_command_index + 1) % len(self.command_sequence)
                    self.last_command_change = current_time
                
                # 接收数据
                self.receive_data()
                
                # 控制循环频率
                time.sleep(0.01)  # 100Hz
                
        except KeyboardInterrupt:
            print("\n检测到Ctrl+C，正在停止...")
        finally:
            self.emergency_stop()
    
    def emergency_stop(self):
        """紧急停止（所有电机快刹车）"""
        print("紧急停止：所有电机快刹车")
        for motor_id in self.motors:
            self.send_motor_command(motor_id, 1, 0.0)
            time.sleep(0.1)
    
    def interactive_control(self):
        """交互式控制模式"""
        print("\n进入交互式控制模式")
        print("命令:")
        print("  s [id] [angle] [time] - 控制舵机")
        print("  m [id] [cmd] [speed]  - 控制电机 (cmd: 0=慢刹, 1=快刹, 2=反转, 3=正转)")
        print("  r - 接收数据")
        print("  e - 紧急停止")
        print("  q - 退出")
        
        while True:
            try:
                cmd = input("\n请输入命令: ").strip().split()
                
                if not cmd:
                    continue
                    
                if cmd[0] == 'q':
                    break
                elif cmd[0] == 'e':
                    self.emergency_stop()
                elif cmd[0] == 'r':
                    self.receive_data()
                elif cmd[0] == 's' and len(cmd) >= 3:
                    servo_id = int(cmd[1])
                    angle = int(cmd[2])
                    duration = int(cmd[3]) if len(cmd) > 3 else 500
                    self.send_servo_command(servo_id, angle, duration)
                elif cmd[0] == 'm' and len(cmd) >= 4:
                    motor_id = int(cmd[1])
                    command = int(cmd[2])
                    speed = float(cmd[3])
                    self.send_motor_command(motor_id, command, speed)
                else:
                    print("未知命令")
                    
            except (ValueError, IndexError):
                print("命令格式错误")
            except KeyboardInterrupt:
                print("\n退出交互模式")
                break

def main():
    """主函数"""
    print("=== 机器人控制器 ===")
    
    # 创建控制器实例
    # 可以尝试不同的端口：
    # - Jetson: /dev/ttyTHS1
    # - USB转串口: /dev/ttyUSB0, /dev/ttyUSB1
    # - Arduino: /dev/ttyACM0
    # - Windows: COM3, COM4
    controller = RobotController(port='/dev/ttyTHS1')
    
    # 连接到串口
    if not controller.connect():
        print("连接失败，请检查:")
        print("1. 设备是否连接")
        print("2. 串口权限是否正确 (sudo chmod 666 /dev/tty*)")
        print("3. 其他程序是否占用了串口")
        return
    
    try:
        # 初始化机器人
        controller.initialize()
        
        # 选择运行模式
        print("\n选择运行模式:")
        print("1. 自动运行电机序列")
        print("2. 交互式控制")
        choice = input("请输入选择 (1或2): ").strip()
        
        if choice == '1':
            controller.run_motor_sequence()
        elif choice == '2':
            controller.interactive_control()
        else:
            print("无效选择，使用自动模式")
            controller.run_motor_sequence()
            
    finally:
        # 确保安全关闭
        controller.emergency_stop()
        controller.disconnect()
        print("程序结束")

if __name__ == "__main__":
    main()