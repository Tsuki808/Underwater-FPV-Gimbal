# gimbal_controller.py

import socket
import struct
import time
import serial # 用于桌面串口测试

# --- Part 1: OpenTrack UDP Receiver ---
# OpenTrack配置的IP和端口，必须完全一致
UDP_IP = "127.0.0.1"
UDP_PORT = 4242

# 创建UDP Socket
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind((UDP_IP, UDP_PORT))
print(f"正在监听来自OpenTrack的数据 (端口: {UDP_PORT})...")


# --- Part 2: Gimbal Private Protocol Implementation ---
# 《云台私有协议》中提供的CRC16校验算法的Python实现
def calculate_crc16(data: bytearray):
    """
    根据先飞机器人提供的C代码实现的CRC-16/X-25校验。
    """
    crc_ta = [
        0x0000, 0x1021, 0x2042, 0x3063, 0x4084, 0x50a5, 0x60c6, 0x70e7,
        0x8108, 0x9129, 0xa14a, 0xb16b, 0xc18c, 0xd1ad, 0xe1ce, 0xf1ef
    ]
    crc = 0
    for byte in data:
        da = (crc >> 12) & 0x0F
        crc = (crc << 4) & 0xFFFF
        crc ^= crc_ta[da ^ (byte >> 4)]
        
        da = (crc >> 12) & 0x0F
        crc = (crc << 4) & 0xFFFF
        crc ^= crc_ta[da ^ (byte & 0x0F)]
        
    return crc & 0xFFFF

def create_gimbal_command(pitch_deg, yaw_deg, roll_deg=0.0, cmd_code=4):
    """
    创建并封装C-20D云台的角度控制指令。

    根据《云台私有协议V1.0.2》实现：
    - 数据包总长度: 40字节
    - 角度单位: 0.01度
    - 控制模式: FPV模式 + 角度控制
    - 字节序: 除CRC外均为小端序，CRC为大端序

    cmd_code: 1=陀螺仪校准, 2=启动云台, 3=停止云台, 4=手动控制
    """
    # 1. 角度转换: 浮点度数 -> 0.01度单位的16位有符号整数
    # 注意: 需要根据实际测试调整正负号
    pitch_val = int(-pitch_deg * 100) # 向上为正（OpenTrack），需测试是否需要取反
    yaw_val = int(yaw_deg * 100)      # 右转为正
    roll_val = int(roll_deg * 100)    # 右倾为正

    # 2. 构建指令包主体 (不含CRC，共38字节)
    # 结构体: Gcu2GbcPkt_t
    packet_body = bytearray()

    # sync[2]: 协议头 (2字节)
    packet_body.extend(b'\xA9\x5B')

    # cmd: 命令字节 (1字节: trig[3位] + valu[5位])
    cmd_byte = (0b000 << 5) | cmd_code
    packet_body.append(cmd_byte)

    # aux: 辅助字节 (1字节: reserved[3位] + fl_sens[5位])
    # fl_sens: FPV跟随灵敏度 [-16,15]，暂设为0
    aux_byte = 0
    packet_body.append(aux_byte)

    # gbc[3]: 3轴控制数据 (9字节 = 3轴 × 3字节/轴)
    # 顺序: gbc[0]=Roll, gbc[1]=Pitch, gbc[2]=Yaw
    axes_data = [roll_val, pitch_val, yaw_val]
    for axis_val in axes_data:
        # 控制字节 (1字节: reserved[3位] + go_zero[1位] + wk_mode[2位] + op_type[2位])
        # op_type=0 (角度控制), wk_mode=0 (Follow模式-更快响应), go_zero=0
        control_byte = (0b000 << 5) | (0b0 << 4) | (0b00 << 2) | 0b00
        packet_body.append(control_byte)
        # op_valu: 16位有符号短整型, 小端序 (2字节)
        packet_body.extend(struct.pack('<h', axis_val))

    # uav: 载机数据 (13字节)
    # 格式: valid[1位]+reserved[7位] + angle[3*2字节] + accel[3*2字节]
    # 暂时用0填充，valid=0表示数据无效
    packet_body.extend(b'\x00' * 13)

    # cam: 相机数据 (12字节)
    # 格式: vert_fov1x[7位]+zoom_value[24位]+reserved[1位] + target_angle[2*4字节]
    # 暂时用0填充
    packet_body.extend(b'\x00' * 12)

    # 3. 计算CRC (校验前38字节)
    crc = calculate_crc16(packet_body)

    # 4. 组合成最终数据包 (40字节)
    # 重要: CRC是协议中唯一使用大端序的字段！
    final_packet = packet_body + struct.pack('>H', crc)

    return final_packet


# --- Part 3: Desktop Test Setup (using USB-to-TTL Serial) ---
# 桌面测试时，请将云台的UART口通过USB-to-TTL模块连接到电脑
# 在设备管理器中查看你的COM口号
try:
    # 修改为你的COM口号, 波特率必须是115200
    ser = serial.Serial('COM12', 115200, timeout=0.1)
    print("串口COM12已打开，准备发送指令...")
except serial.SerialException as e:
    print(f"无法打开串口: {e}")
    print("请确认连接了USB-to-TTL模块，并修改了正确的COM口号。")
    ser = None


# --- Main Loop ---
if __name__ == "__main__":
    print("\n=== 云台控制器已启动 ===")
    print(f"数据包长度: 40字节")
    print(f"控制频率: 100Hz (每10ms一次)")
    print(f"控制模式: Follow模式 + 角度控制")
    print("按 Ctrl+C 退出\n")

    # 发送云台启动命令
    if ser and ser.is_open:
        print(">>> 正在发送云台启动命令 (cmd=2)...")
        startup_cmd = create_gimbal_command(0, 0, 0, cmd_code=2)
        ser.write(startup_cmd)
        time.sleep(0.5)

        # 尝试读取云台响应
        if ser.in_waiting > 0:
            response = ser.read(ser.in_waiting)
            print(f"云台响应 ({len(response)}字节): {response.hex(' ')}")
        else:
            print("⚠️  警告: 未收到云台响应！请检查：")
            print("   1. USB-TTL的RXD是否连接到云台的UART_Tx")
            print("   2. 云台是否已上电")
            print("   3. 串口号是否正确\n")

    gimbal_started = False

    while True:
        try:
            # 接收数据 (OpenTrack发送的数据包大小固定为48字节)
            data, addr = sock.recvfrom(48)

            # 数据包长度校验
            if len(data) != 48:
                print(f"⚠️  警告: 收到异常长度的UDP数据包 ({len(data)}字节，期望48字节)")
                continue

            # 解包数据，我们只需要Yaw, Pitch, Roll
            # 格式是6个double (8字节), 我们要的是后3个
            # '<' 代表小端序, 'd' 代表double
            raw_yaw = struct.unpack_from('<d', data, 24)[0]
            raw_pitch = struct.unpack_from('<d', data, 32)[0]
            raw_roll = struct.unpack_from('<d', data, 40)[0]

            # 打印原始数据用于调试
            print(f"Raw -> Yaw:{raw_yaw:6.2f}° Pitch:{raw_pitch:6.2f}° Roll:{raw_roll:6.2f}°", end=' | ')

            # 生成云台指令
            command = create_gimbal_command(raw_pitch, raw_yaw, raw_roll, cmd_code=4)

            # 验证数据包长度
            if len(command) != 40:
                print(f"\n❌ 错误: 生成的数据包长度异常 ({len(command)}字节，期望40字节)")
                continue

            # 每隔一段时间打印完整数据包
            if not gimbal_started:
                print(f"Pkt: {command[:16].hex(' ')}...")
                gimbal_started = True
            else:
                print(f"Pkt: {command[:6].hex(' ')}...")

            # 如果串口打开了，就发送指令
            if ser and ser.is_open:
                bytes_written = ser.write(command)
                if bytes_written != 40:
                    print(f"⚠️  警告: 串口写入不完整 ({bytes_written}/40字节)")

                # 读取云台返回数据（如果有）
                if ser.in_waiting > 0:
                    response = ser.read(ser.in_waiting)
                    if len(response) == 26:  # 云台返回数据包是26字节
                        print(f" [云台响应: {response.hex(' ')}]", end='')

            # 控制发送频率 (100Hz -> 0.01s = 10ms，提高响应速度)
            time.sleep(0.01)

        except KeyboardInterrupt:
            print("\n\n=== 程序已停止 ===")
            if ser and ser.is_open:
                ser.close()
                print("串口已关闭")
            sock.close()
            print("UDP Socket已关闭")
            break
        except Exception as e:
            print(f"\n❌ 发生错误: {e}")
            import traceback
            traceback.print_exc()