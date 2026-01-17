import serial
import time

class MotorController:
    def __init__(self, port='/dev/ttyTHS1', baudrate=115200, timeout=1):
        """
        电机控制器

        Args:
            port: 串口端口
            baudrate: 波特率
            timeout: 超时时间
        """
        self.serial_port = None
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout

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

    def parse_command(self, cmd_input):
        """
        解析命令输入

        Args:
            cmd_input: 用户输入的命令字符串
        """
        parts = cmd_input.strip().split()
        if len(parts) != 3:
            print("命令格式错误！请输入: 电机ID 命令 速度")
            print("例如: 0 3 100 (电机0正转，速度100 RPM)")
            print("命令类型: 0=慢刹车, 1=快刹车, 2=反转, 3=正转")
            return None

        try:
            motor_id = int(parts[0])
            command = int(parts[1])
            speed = float(parts[2])

            if not (0 <= motor_id <= 3):
                print("电机ID应在0-3之间")
                return None
            if not (0 <= command <= 3):
                print("命令应在0-3之间 (0=慢刹车, 1=快刹车, 2=反转, 3=正转)")
                return None

            return motor_id, command, speed
        except ValueError:
            print("参数类型错误！请输入数字")
            return None

    def show_help(self):
        """显示帮助信息"""
        print("\n=== 电机控制命令帮助 ===")
        print("命令格式: 电机ID 命令 速度")
        print("电机ID: 0-3 (对应物理电机1-4)")
        print("命令类型:")
        print("  0 - 慢刹车")
        print("  1 - 快刹车")
        print("  2 - 反转")
        print("  3 - 正转")
        print("示例:")
        print("  0 3 100   - 电机0正转，速度100 RPM")
        print("  1 2 50    - 电机1反转，速度50 RPM")
        print("  2 1 0     - 电机2快刹车")
        print("  h         - 显示此帮助")
        print("  q         - 退出程序")
        print("========================\n")

    def emergency_stop(self):
        """紧急停止（所有电机快刹车）"""
        print("紧急停止：所有电机快刹车")
        for motor_id in range(4):
            self.send_motor_command(motor_id, 1, 0.0)
            time.sleep(0.1)


def main():
    """主函数"""
    print("=== 电机控制器 ===")
    print("命令格式: 电机ID 命令 速度")
    print("输入 'h' 查看帮助，输入 'q' 退出")
    
    # 创建控制器实例
    controller = MotorController(port='/dev/ttyTHS1')

    # 连接到串口
    if not controller.connect():
        print("连接失败，请检查:")
        print("1. 设备是否连接")
        print("2. 串口权限是否正确 (sudo chmod 666 /dev/tty*)")
        print("3. 其他程序是否占用了串口")
        return

    try:
        print("\n连接成功！")
        controller.show_help()

        while True:
            try:
                user_input = input("请输入命令: ").strip()

                if user_input.lower() == 'q':
                    print("退出程序...")
                    break
                elif user_input.lower() == 'h':
                    controller.show_help()
                elif user_input.lower() == 'e':
                    controller.emergency_stop()
                else:
                    # 解析并执行命令
                    result = controller.parse_command(user_input)
                    if result:
                        motor_id, command, speed = result
                        controller.send_motor_command(motor_id, command, speed)
                    else:
                        print("命令解析失败，请重新输入")

            except KeyboardInterrupt:
                print("\n收到中断信号，退出程序...")
                break
            except Exception as e:
                print(f"执行命令时出错: {e}")

    finally:
        # 程序退出前紧急停止
        print("\n程序退出，执行紧急停止...")
        controller.emergency_stop()
        # 断开连接
        controller.disconnect()
        print("程序结束")


if __name__ == "__main__":
    main()