import serial
import time
import math
import threading
import queue

class MotorSequenceController:
    def __init__(self, port='/dev/ttyTHS1', baudrate=115200, timeout=1):
        """
        电机序列控制器

        Args:
            port: 串口端口
            baudrate: 波特率
            timeout: 超时时间
        """
        self.serial_port = None
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self.is_running = False
        self.thread = None
        self.data_queue = queue.Queue()
        
        # 电机ID列表
        self.motors = [0, 1, 2, 3]  # 对应物理电机1-4

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
        self.is_running = False
        if self.thread and self.thread.is_alive():
            self.thread.join(timeout=2)
        
        if self.serial_port and self.serial_port.is_open:
            self.serial_port.close()
            print("串口已关闭")

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
            cmd_names = ["慢刹车", "快刹车", "反转", "正转"]
            print(f"发送电机{motor_id+1}命令: {command_str.strip()} ({cmd_names[command]}, {speed_rpm} RPM)")
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
                if len(parts) >= 32:  # POSE,16个舵机角度,时间戳,6个IMU值,温度,4个电机转向,4个电机转速
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
                    motor_directions = [int(parts[i]) for i in range(25, 29)]  # 4个电机转向
                    motor_speeds = [float(parts[i]) for i in range(29, 33)]    # 4个电机转速

                    result = {
                        'servo_angles': servo_angles,
                        'timestamp': timestamp,
                        'accel': (accel_x, accel_y, accel_z),
                        'gyro': (gyro_x, gyro_y, gyro_z),
                        'temperature': temperature,
                        'motor_directions': motor_directions,
                        'motor_speeds': motor_speeds
                    }
                    return result
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

    def data_receiver_thread(self):
        """数据接收线程"""
        while self.is_running:
            try:
                results = self.receive_data()
                if results:
                    for result in results:
                        self.data_queue.put(result)
                time.sleep(0.01)  # 100Hz
            except Exception as e:
                print(f"数据接收线程出错: {e}")
                time.sleep(0.1)

    def print_pose_data(self, data):
        """打印姿态数据"""
        print(f"\n=== 姿态数据 ===")
        print(f"时间戳: {data['timestamp']}ms")
        print(f"舵机角度: {data['servo_angles']}")
        print(f"加速度: ({data['accel'][0]:.2f}, {data['accel'][1]:.2f}, {data['accel'][2]:.2f}) m/s²")
        print(f"陀螺仪: ({data['gyro'][0]:.2f}, {data['gyro'][1]:.2f}, {data['gyro'][2]:.2f}) °/s")
        print(f"温度: {data['temperature']:.1f}°C")
        print(f"电机转向: {data['motor_directions']}")
        print(f"电机转速: {data['motor_speeds']}")
        print("=" * 30)

    def gradual_speed_change(self, motor_id, start_speed, end_speed, duration):
        """
        平滑改变电机转速

        Args:
            motor_id: 电机ID
            start_speed: 起始转速
            end_speed: 结束转速
            duration: 变化时间（秒）
        """
        steps = int(duration * 10)  # 每0.1秒更新一次
        step_time = duration / steps
        
        for i in range(steps + 1):
            ratio = i / steps
            current_speed = start_speed + (end_speed - start_speed) * ratio
            
            # 确定命令类型
            if abs(current_speed) < 0.1:  # 接近0
                command = 1  # 快刹车
                speed = 0.0
            elif current_speed > 0:
                command = 3  # 正转
                speed = abs(current_speed)
            else:
                command = 2  # 反转
                speed = abs(current_speed)
            
            self.send_motor_command(motor_id, command, speed)
            
            # 检查是否有新的姿态数据
            try:
                while not self.data_queue.empty():
                    pose_data = self.data_queue.get_nowait()
                    self.print_pose_data(pose_data)
            except:
                pass
            
            time.sleep(step_time)

    def motor_sequence_thread(self):
        """电机序列控制线程"""
        print("开始电机序列控制...")
        
        while self.is_running:
            try:
                # 1. 正转 380rpm
                print("步骤1: 正转 380rpm")
                for motor_id in self.motors:
                    self.send_motor_command(motor_id, 3, 380.0)  # 正转380rpm
                time.sleep(1)  # 稳定1秒
                
                # 检查姿态数据
                try:
                    while not self.data_queue.empty():
                        pose_data = self.data_queue.get_nowait()
                        self.print_pose_data(pose_data)
                except:
                    pass
                
                # 2. 在5秒钟里转速缓慢下降到0
                print("步骤2: 5秒内转速从380rpm平滑下降到0")
                self.gradual_speed_change(0, 380.0, 0.0, 5.0)
                self.gradual_speed_change(1, 380.0, 0.0, 5.0)
                self.gradual_speed_change(2, 380.0, 0.0, 5.0)
                self.gradual_speed_change(3, 380.0, 0.0, 5.0)
                
                # 3. 5秒钟内缓慢上升到反转380rpm
                print("步骤3: 5秒内转速从0平滑上升到反转380rpm")
                self.gradual_speed_change(0, 0.0, -380.0, 5.0)
                self.gradual_speed_change(1, 0.0, -380.0, 5.0)
                self.gradual_speed_change(2, 0.0, -380.0, 5.0)
                self.gradual_speed_change(3, 0.0, -380.0, 5.0)
                
                # 4. 快刹5秒
                print("步骤4: 快刹5秒")
                for motor_id in self.motors:
                    self.send_motor_command(motor_id, 1, 0.0)  # 快刹车
                time.sleep(5)
                
                # 5. 慢刹5秒
                print("步骤5: 慢刹5秒")
                for motor_id in self.motors:
                    self.send_motor_command(motor_id, 0, 0.0)  # 慢刹车
                time.sleep(5)
                
                # 6. 5秒里转速缓慢上升到正转390rpm
                print("步骤6: 5秒内转速从0平滑上升到正转390rpm")
                self.gradual_speed_change(0, 0.0, 390.0, 5.0)
                self.gradual_speed_change(1, 0.0, 390.0, 5.0)
                self.gradual_speed_change(2, 0.0, 390.0, 5.0)
                self.gradual_speed_change(3, 0.0, 390.0, 5.0)
                
                # 检查姿态数据
                try:
                    while not self.data_queue.empty():
                        pose_data = self.data_queue.get_nowait()
                        self.print_pose_data(pose_data)
                except:
                    pass
                
                # 循环等待
                print("序列完成，等待下一循环...")
                time.sleep(2)  # 短暂等待后重新开始循环
                
            except Exception as e:
                print(f"电机序列执行出错: {e}")
                if not self.is_running:
                    break
                time.sleep(1)

    def start_sequence(self):
        """开始电机序列控制"""
        if not self.serial_port or not self.serial_port.is_open:
            print("错误: 串口未连接")
            return False

        # 启动数据接收线程
        self.is_running = True
        receiver_thread = threading.Thread(target=self.data_receiver_thread)
        receiver_thread.daemon = True
        receiver_thread.start()
        
        # 启动电机序列线程
        sequence_thread = threading.Thread(target=self.motor_sequence_thread)
        sequence_thread.daemon = True
        sequence_thread.start()
        
        print("电机序列控制已启动")
        return True

    def stop_sequence(self):
        """停止电机序列控制"""
        self.is_running = False
        print("电机序列控制已停止")

    def emergency_stop(self):
        """紧急停止（所有电机快刹车）"""
        print("紧急停止：所有电机快刹车")
        for motor_id in self.motors:
            self.send_motor_command(motor_id, 1, 0.0)
            time.sleep(0.1)

    def interactive_monitor(self):
        """交互式监控模式"""
        print("进入交互式监控模式...")
        print("按 Enter 键查看最新姿态数据，或输入 'q' 退出")
        
        while True:
            try:
                user_input = input()
                if user_input.lower() == 'q':
                    break
                
                # 显示最新的姿态数据
                latest_data = None
                try:
                    while not self.data_queue.empty():
                        latest_data = self.data_queue.get_nowait()
                except:
                    pass
                
                if latest_data:
                    self.print_pose_data(latest_data)
                else:
                    print("暂无姿态数据")
                    
            except KeyboardInterrupt:
                print("\n退出监控模式")
                break


def main():
    """主函数"""
    print("=== 电机序列控制器 ===")
    print("控制序列: 正转380rpm -> (5秒降速) -> 0 -> (5秒升速) -> 反转380rpm -> 快刹5秒 -> 慢刹5秒 -> (5秒升速) -> 正转390rpm")
    
    # 创建控制器实例
    controller = MotorSequenceController(port='/dev/ttyTHS1')

    # 连接到串口
    if not controller.connect():
        print("连接失败，请检查:")
        print("1. 设备是否连接")
        print("2. 串口权限是否正确 (sudo chmod 666 /dev/tty*)")
        print("3. 其他程序是否占用了串口")
        return

    try:
        print("\n程序启动选项:")
        print("1. 开始电机序列控制")
        print("2. 交互式控制")
        print("3. 交互式监控")
        print("4. 退出")
        
        choice = input("请选择 (1/2/3/4): ").strip()

        if choice == '1':
            print("启动电机序列控制...")
            controller.start_sequence()
            
            print("\n控制已启动，按 Ctrl+C 停止")
            try:
                while True:
                    # 检查是否有新的姿态数据
                    try:
                        while not controller.data_queue.empty():
                            pose_data = controller.data_queue.get_nowait()
                            controller.print_pose_data(pose_data)
                    except:
                        pass
                    time.sleep(0.1)  # 让主线程保持活跃
            except KeyboardInterrupt:
                print("\n收到停止信号...")
                
        elif choice == '2':
            print("交互式控制模式:")
            print("命令: m [motor_id] [command] [speed] - 控制电机")
            print("       e - 紧急停止")
            print("       q - 退出")
            
            while True:
                cmd_input = input("请输入命令: ").strip().split()
                
                if not cmd_input:
                    continue
                    
                if cmd_input[0] == 'q':
                    break
                elif cmd_input[0] == 'e':
                    controller.emergency_stop()
                elif cmd_input[0] == 'm' and len(cmd_input) >= 4:
                    try:
                        motor_id = int(cmd_input[1])
                        command = int(cmd_input[2])
                        speed = float(cmd_input[3])
                        
                        if 0 <= motor_id <= 3 and 0 <= command <= 3:
                            controller.send_motor_command(motor_id, command, speed)
                        else:
                            print("电机ID应为0-3，命令应为0-3")
                    except ValueError:
                        print("参数错误，请输入: m [motor_id] [command] [speed]")
                else:
                    print("未知命令")
                    
        elif choice == '3':
            # 启动数据接收线程
            controller.is_running = True
            receiver_thread = threading.Thread(target=controller.data_receiver_thread)
            receiver_thread.daemon = True
            receiver_thread.start()
            
            controller.interactive_monitor()
            
        else:
            print("退出程序")

    finally:
        # 停止序列控制
        controller.stop_sequence()
        # 紧急停止
        controller.emergency_stop()
        # 断开连接
        controller.disconnect()
        print("程序结束")


if __name__ == "__main__":
    main()