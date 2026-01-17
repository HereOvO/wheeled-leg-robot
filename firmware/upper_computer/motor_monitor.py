import serial
import time
import threading
import queue

class MotorMonitor:
    def __init__(self, port='/dev/ttyTHS1', baudrate=115200, timeout=1):
        """
        电机速度监视器

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
        self.data_queue = queue.Queue()
        self.latest_data = None

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
        if self.serial_port and self.serial_port.is_open:
            self.serial_port.close()
            print("串口已关闭")

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
                        self.latest_data = result
                time.sleep(0.01)  # 100Hz
            except Exception as e:
                print(f"数据接收线程出错: {e}")
                time.sleep(0.1)

    def print_motor_speeds(self, data):
        """打印电机速度"""
        speeds = data['motor_speeds']
        directions = data['motor_directions']
        
        print(f"\n=== 电机速度监视 ===")
        print(f"时间戳: {data['timestamp']}ms")
        print(f"电机1: {speeds[0]:.2f} RPM (方向: {directions[0]})")
        print(f"电机2: {speeds[1]:.2f} RPM (方向: {directions[1]})")
        print(f"电机3: {speeds[2]:.2f} RPM (方向: {directions[2]})")
        print(f"电机4: {speeds[3]:.2f} RPM (方向: {directions[3]})")
        print("=" * 30)

    def start_monitoring(self):
        """开始监视电机速度"""
        if not self.serial_port or not self.serial_port.is_open:
            print("错误: 串口未连接")
            return False

        self.is_running = True
        
        # 启动数据接收线程
        receiver_thread = threading.Thread(target=self.data_receiver_thread)
        receiver_thread.daemon = True
        receiver_thread.start()
        
        print("开始监视电机速度... 按 Ctrl+C 停止")
        
        try:
            while self.is_running:
                try:
                    # 检查是否有新的数据
                    while not self.data_queue.empty():
                        pose_data = self.data_queue.get_nowait()
                        self.print_motor_speeds(pose_data)
                    
                    time.sleep(0.1)  # 每0.1秒检查一次
                except KeyboardInterrupt:
                    print("\n停止监视...")
                    break
        except Exception as e:
            print(f"监视过程中出错: {e}")
        finally:
            self.is_running = False

    def get_latest_motor_data(self):
        """获取最新的电机数据"""
        return self.latest_data


def main():
    """主函数"""
    print("=== 电机速度监视器 ===")
    
    # 创建监视器实例
    monitor = MotorMonitor(port='/dev/ttyTHS1')

    # 连接到串口
    if not monitor.connect():
        print("连接失败，请检查:")
        print("1. 设备是否连接")
        print("2. 串口权限是否正确 (sudo chmod 666 /dev/tty*)")
        print("3. 其他程序是否占用了串口")
        return

    try:
        print("\n开始监视电机速度...")
        monitor.start_monitoring()
    finally:
        # 断开连接
        monitor.disconnect()
        print("程序结束")


if __name__ == "__main__":
    main()