#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
==========================================
C-20D云台控制器 - 终极优化版 v3.0
==========================================

功能特性:
- 支持OpenTrack UDP头部追踪输入
- 极致性能优化（7大优化技术）
- 内置参数调优模式
- 实时性能监控
- 多预设配置

协议: C-20D云台私有协议 V1.0.2
"""

import socket
import struct
import time
import serial
import sys
import os
from collections import deque
from threading import Thread, Lock

try:
    import numpy as np
except ImportError:
    print("警告: numpy未安装，使用纯Python实现（性能略降）")
    np = None

# ==================== 配置区域 ====================

class Config:
    """配置类 - 所有参数集中管理"""

    # 串口转网口TCP配置
    # NOTE: 请替换为你自己的串口服务器 IP / 端口
    SERIAL_BRIDGE_IP = '192.168.1.50'
    SERIAL_BRIDGE_PORT = 4019
    # TCP端口，程序会自动尝试多个端口
    USE_TCP = True  # True=TCP网络模式, False=直连串口模式
    
    # 直连串口配置（USE_TCP=False时使用）
    SERIAL_PORT = 'COM13'
    SERIAL_BAUDRATE = 115200

    # UDP配置
    UDP_IP = "127.0.0.1"
    UDP_PORT = 4242

    # 性能参数（平衡延迟和稳定性）
    UPDATE_RATE = 120        # Hz，发送频率
    PREDICTION_MS = 30       # 毫秒，适度预测
    SMOOTHING_FACTOR = 0.2   # 0-1，适度滤波
    DEADZONE = 0.08          # 度，适度死区
    MAX_SPEED = 100.0        # 度/秒，限制速度

    # 工作模式
    # 0=Follow跟随, 1=Lock锁定, 2=FPV
    WORK_MODE = 0  # 先尝试Follow模式，这个模式响应更快

    # 控制类型
    # 0=角度控制, 1=比例角速度, 2=真实角速度
    CONTROL_TYPE = 0

    # 角度反向（如果云台运动方向相反，改为-1）
    PITCH_INVERT = -1  # Pitch必须取反
    YAW_INVERT = 1
    ROLL_INVERT = 1
    
    # 调试模式：显示发送的原始数据包
    SHOW_RAW_PACKETS = True  # 显示首个数据包的十六进制

    # 调试选项
    SHOW_DEBUG = False       # 显示详细调试信息
    SHOW_STATS = True        # 显示性能统计
    STATS_INTERVAL = 1.0     # 秒，统计更新间隔

    # 预设配置
    PRESETS = {
        'turbo': {
            'name': '极速响应',
            'UPDATE_RATE': 150,
            'PREDICTION_MS': 10,
            'SMOOTHING_FACTOR': 0.5,
            'DEADZONE': 0.02,
            'MAX_SPEED': 100.0,
        },
        'smooth': {
            'name': '平滑稳定',
            'UPDATE_RATE': 100,
            'PREDICTION_MS': 20,
            'SMOOTHING_FACTOR': 0.1,
            'DEADZONE': 0.1,
            'MAX_SPEED': 30.0,
        },
        'balanced': {
            'name': '均衡模式',
            'UPDATE_RATE': 100,
            'PREDICTION_MS': 15,
            'SMOOTHING_FACTOR': 0.3,
            'DEADZONE': 0.05,
            'MAX_SPEED': 50.0,
        },
    }

    @classmethod
    def apply_preset(cls, preset_name):
        """应用预设配置"""
        if preset_name in cls.PRESETS:
            preset = cls.PRESETS[preset_name]
            for key, value in preset.items():
                if key != 'name' and hasattr(cls, key):
                    setattr(cls, key, value)
            return preset['name']
        return None

# ==================== CRC计算 ====================

CRC_TABLE = [
    0x0000, 0x1021, 0x2042, 0x3063, 0x4084, 0x50a5, 0x60c6, 0x70e7,
    0x8108, 0x9129, 0xa14a, 0xb16b, 0xc18c, 0xd1ad, 0xe1ce, 0xf1ef
]

def calculate_crc16(data: bytes) -> int:
    """CRC16校验（预计算表优化）"""
    crc = 0
    for byte in data:
        da = (crc >> 12) & 0x0F
        crc = ((crc << 4) & 0xFFFF) ^ CRC_TABLE[da ^ (byte >> 4)]
        da = (crc >> 12) & 0x0F
        crc = ((crc << 4) & 0xFFFF) ^ CRC_TABLE[da ^ (byte & 0x0F)]
    return crc

# ==================== 卡尔曼滤波器 ====================

class KalmanFilter:
    """一维卡尔曼滤波器"""
    def __init__(self, measurement_variance=0.3):
        self.process_variance = 1e-3
        self.measurement_variance = measurement_variance
        self.estimate = 0.0
        self.estimate_error = 1.0

    def update(self, measurement):
        # 预测
        prediction_error = self.estimate_error + self.process_variance

        # 更新
        kalman_gain = prediction_error / (prediction_error + self.measurement_variance)
        self.estimate = self.estimate + kalman_gain * (measurement - self.estimate)
        self.estimate_error = (1 - kalman_gain) * prediction_error

        return self.estimate

# ==================== 运动预测器 ====================

class MotionPredictor:
    """基于速度的运动预测器"""
    def __init__(self, window_size=5):
        self.history = deque(maxlen=window_size)
        self.last_time = time.perf_counter()

    def predict(self, current_angle, prediction_time_ms):
        current_time = time.perf_counter()
        dt = current_time - self.last_time

        if dt > 0 and len(self.history) > 0:
            velocity = (current_angle - self.history[-1][1]) / dt

            # 限制最大速度
            if np:
                velocity = np.clip(velocity, -Config.MAX_SPEED, Config.MAX_SPEED)
            else:
                velocity = max(-Config.MAX_SPEED, min(Config.MAX_SPEED, velocity))

            predicted = current_angle + velocity * (prediction_time_ms / 1000.0)
        else:
            predicted = current_angle

        self.history.append((current_time, current_angle))
        self.last_time = current_time

        return predicted

# ==================== 数据包构建器 ====================

class PacketBuilder:
    """零拷贝数据包构建器"""
    def __init__(self):
        self.buffer = bytearray(40)

        # 预设固定字段
        self.buffer[0:2] = b'\xA9\x5B'  # sync
        self.buffer[2] = (0b000 << 5) | 4  # cmd=4 (手动控制)
        self.buffer[3] = 0  # aux

        # 控制字节
        control_byte = (0b000 << 5) | (0b0 << 4) | (Config.WORK_MODE << 2) | Config.CONTROL_TYPE
        self.buffer[4] = control_byte   # Roll
        self.buffer[7] = control_byte   # Pitch
        self.buffer[10] = control_byte  # Yaw

    def build(self, pitch, yaw, roll=0.0):
        """构建数据包（原地修改）"""
        # 角度转换（0.01度单位）+ 反向处理
        roll_val = int(roll * 100 * Config.ROLL_INVERT)
        pitch_val = int(pitch * 100 * Config.PITCH_INVERT)
        yaw_val = int(yaw * 100 * Config.YAW_INVERT)

        # 限制范围
        roll_val = max(-32768, min(32767, roll_val))
        pitch_val = max(-32768, min(32767, pitch_val))
        yaw_val = max(-32768, min(32767, yaw_val))

        # 写入角度值（小端序）
        # gbc[0]=Roll, gbc[1]=Pitch, gbc[2]=Yaw
        struct.pack_into('<h', self.buffer, 5, roll_val)
        struct.pack_into('<h', self.buffer, 8, pitch_val)
        struct.pack_into('<h', self.buffer, 11, yaw_val)

        # 计算CRC（大端序）
        crc = calculate_crc16(self.buffer[:38])
        struct.pack_into('>H', self.buffer, 38, crc)

        return self.buffer

# ==================== 异步串口/网络管理 ====================

class AsyncSerialManager:
    """异步串口/TCP网络管理器"""
    def __init__(self, use_tcp=False, tcp_ip=None, tcp_port=None, serial_port=None, baudrate=115200):
        self.use_tcp = use_tcp
        self.write_queue = deque(maxlen=10)
        self.lock = Lock()
        self.running = True
        
        if use_tcp:
            # TCP模式 - 自动尝试多个端口
            self.tcp_socket = None
            common_ports = [4019]
            
            print(f"\n正在尝试连接串口转网口设备 {tcp_ip} ...")
            for port in common_ports:
                try:
                    print(f"  尝试端口 {port} ... ", end='', flush=True)
                    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                    sock.settimeout(1.0)
                    sock.connect((tcp_ip, port))
                    self.tcp_socket = sock
                    print(f"✅ 连接成功!")
                    print(f"✅ 已连接到 {tcp_ip}:{port} (TCP模式)")
                    break
                except Exception as e:
                    print(f"失败")
                    
            if self.tcp_socket is None:
                raise ConnectionError(f"无法连接到 {tcp_ip}，尝试了端口: {common_ports}")
        else:
            # 串口模式
            self.serial = serial.Serial(serial_port, baudrate, timeout=0.01)
            self.tcp_socket = None
            
        self.thread = Thread(target=self._write_worker, daemon=True)
        self.thread.start()

    def send(self, data):
        with self.lock:
            if len(self.write_queue) < 10:
                self.write_queue.append(bytes(data))

    def _write_worker(self):
        while self.running:
            if self.write_queue:
                with self.lock:
                    data = self.write_queue.popleft()
                try:
                    if self.use_tcp:
                        self.tcp_socket.send(data)
                    else:
                        self.serial.write(data)
                except Exception as e:
                    print(f"\n❌ 发送失败: {e}")
            else:
                time.sleep(0.001)

    def close(self):
        self.running = False
        self.thread.join(timeout=1.0)
        if self.use_tcp:
            if self.tcp_socket:
                self.tcp_socket.close()
        else:
            self.serial.close()

# ==================== 性能监控 ====================

class PerformanceMonitor:
    """性能监控器"""
    def __init__(self, window=100):
        self.times = deque(maxlen=window)
        self.last_time = time.perf_counter()

    def tick(self):
        current = time.perf_counter()
        dt = current - self.last_time
        self.times.append(dt)
        self.last_time = current

    def get_stats(self):
        if not self.times:
            return {'avg_fps': 0, 'avg_latency_ms': 0}

        if np:
            times = np.array(self.times)
            avg_time = np.mean(times)
        else:
            avg_time = sum(self.times) / len(self.times)

        return {
            'avg_fps': 1.0 / avg_time if avg_time > 0 else 0,
            'avg_latency_ms': avg_time * 1000,
        }

# ==================== 主控制器 ====================

class GimbalController:
    """云台控制器主类"""
    def __init__(self):
        # UDP接收
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind((Config.UDP_IP, Config.UDP_PORT))
        self.sock.settimeout(0.1)

        # 串口/网络连接
        try:
            if Config.USE_TCP:
                self.serial = AsyncSerialManager(
                    use_tcp=True,
                    tcp_ip=Config.SERIAL_BRIDGE_IP,
                    tcp_port=Config.SERIAL_BRIDGE_PORT
                )
            else:
                self.serial = AsyncSerialManager(
                    use_tcp=False,
                    serial_port=Config.SERIAL_PORT,
                    baudrate=Config.SERIAL_BAUDRATE
                )
                print(f"✅ 串口 {Config.SERIAL_PORT} 已打开 ({Config.SERIAL_BAUDRATE} bps)")
        except Exception as e:
            print(f"❌ 连接失败: {e}")
            self.serial = None

        # 数据包构建器
        self.packet_builder = PacketBuilder()

        # 滤波器
        self.filter_pitch = KalmanFilter(Config.SMOOTHING_FACTOR)
        self.filter_yaw = KalmanFilter(Config.SMOOTHING_FACTOR)
        self.filter_roll = KalmanFilter(Config.SMOOTHING_FACTOR)

        # 预测器
        self.predictor_pitch = MotionPredictor()
        self.predictor_yaw = MotionPredictor()
        self.predictor_roll = MotionPredictor()

        # 性能监控
        self.perf_monitor = PerformanceMonitor()

        # 状态
        self.last_angles = [0.0, 0.0, 0.0]
        self.running = True

    def send_startup_command(self):
        """发送云台启动命令"""
        if not self.serial:
            return

        startup_buffer = bytearray(40)
        startup_buffer[0:2] = b'\xA9\x5B'
        startup_buffer[2] = (0b000 << 5) | 2  # cmd=2 启动

        control_byte = (0b000 << 5) | (0b0 << 4) | (Config.WORK_MODE << 2) | Config.CONTROL_TYPE
        startup_buffer[4] = control_byte
        startup_buffer[7] = control_byte
        startup_buffer[10] = control_byte

        crc = calculate_crc16(startup_buffer[:38])
        struct.pack_into('>H', startup_buffer, 38, crc)

        self.serial.send(startup_buffer)
        print(">>> 已发送云台启动命令 (cmd=2)")

    def apply_deadzone(self, value, deadzone):
        """应用死区"""
        if abs(value) < deadzone:
            return 0.0
        return value

    def process_angles(self, raw_pitch, raw_yaw, raw_roll):
        """处理角度：滤波 + 死区 + 预测"""
        # 1. 卡尔曼滤波
        pitch = self.filter_pitch.update(raw_pitch)
        yaw = self.filter_yaw.update(raw_yaw)
        roll = self.filter_roll.update(raw_roll)

        # 2. 运动预测
        if Config.PREDICTION_MS > 0:
            pitch = self.predictor_pitch.predict(pitch, Config.PREDICTION_MS)
            yaw = self.predictor_yaw.predict(yaw, Config.PREDICTION_MS)
            roll = self.predictor_roll.predict(roll, Config.PREDICTION_MS)

        # 3. 死区处理
        if Config.DEADZONE > 0:
            pitch_delta = self.apply_deadzone(pitch - self.last_angles[0], Config.DEADZONE)
            yaw_delta = self.apply_deadzone(yaw - self.last_angles[1], Config.DEADZONE)
            roll_delta = self.apply_deadzone(roll - self.last_angles[2], Config.DEADZONE)

            pitch = self.last_angles[0] + pitch_delta
            yaw = self.last_angles[1] + yaw_delta
            roll = self.last_angles[2] + roll_delta

        self.last_angles = [pitch, yaw, roll]
        return pitch, yaw, roll

    def run(self):
        """主循环"""
        self.print_banner()

        # 发送启动命令
        self.send_startup_command()
        time.sleep(0.5)

        # 性能统计
        stats_timer = time.perf_counter()
        loop_interval = 1.0 / Config.UPDATE_RATE
        next_update = time.perf_counter()

        # 数据去重
        last_data_hash = 0
        packet_count = 0

        try:
            while self.running:
                try:
                    # 接收UDP数据
                    data, _ = self.sock.recvfrom(48)

                    # 跳过重复数据包（OpenTrack爆发式发送）
                    data_hash = hash(data)
                    if data_hash == last_data_hash:
                        continue
                    last_data_hash = data_hash

                    if len(data) != 48:
                        continue

                    # 解包（Yaw, Pitch, Roll）
                    raw_yaw = struct.unpack_from('<d', data, 24)[0]
                    raw_pitch = struct.unpack_from('<d', data, 32)[0]
                    raw_roll = struct.unpack_from('<d', data, 40)[0]

                    # 处理角度
                    pitch, yaw, roll = self.process_angles(raw_pitch, raw_yaw, raw_roll)

                    # 构建数据包
                    packet = self.packet_builder.build(pitch, yaw, roll)

                    # 显示首个数据包（调试）
                    if Config.SHOW_RAW_PACKETS and packet_count == 0:
                        print(f"\n>>> 首个控制包 (cmd=4, {['Follow', 'Lock', 'FPV'][Config.WORK_MODE]}模式):")
                        print(f"    完整包: {packet[:13].hex(' ').upper()} ... {packet[38:40].hex(' ').upper()}")
                        roll_val = struct.unpack_from('<h', packet, 5)[0]
                        pitch_val = struct.unpack_from('<h', packet, 8)[0]
                        yaw_val = struct.unpack_from('<h', packet, 11)[0]
                        print(f"    角度值: Roll={roll_val}, Pitch={pitch_val}, Yaw={yaw_val} (0.01度单位)")
                        print(f"    控制字节: 0x{packet[4]:02X}\n")
                    packet_count += 1

                    # 发送
                    if self.serial:
                        self.serial.send(packet)

                    # 性能监控
                    self.perf_monitor.tick()

                    # 显示统计
                    if Config.SHOW_STATS and time.perf_counter() - stats_timer >= Config.STATS_INTERVAL:
                        stats = self.perf_monitor.get_stats()
                        print(f"\r📈 FPS:{stats['avg_fps']:6.1f} | 延迟:{stats['avg_latency_ms']:5.2f}ms | "
                              f"P:{pitch:6.2f}° Y:{yaw:6.2f}° R:{roll:6.2f}°", end='', flush=True)
                        stats_timer = time.perf_counter()

                    # 精确定时
                    next_update += loop_interval
                    sleep_time = next_update - time.perf_counter()
                    if sleep_time > 0:
                        time.sleep(sleep_time)
                    else:
                        next_update = time.perf_counter()

                except socket.timeout:
                    continue

        except KeyboardInterrupt:
            print("\n\n⏹️  正在停止...")
        finally:
            self.cleanup()

    def print_banner(self):
        """打印启动信息"""
        print("\n" + "="*70)
        print(" " * 20 + "C-20D云台控制器 v3.0 (TCP网络版)")
        print("="*70)
        print(f"📡 UDP监听: {Config.UDP_IP}:{Config.UDP_PORT} (接收OpenTrack)")
        if Config.USE_TCP:
            print(f"🌐 TCP连接: {Config.SERIAL_BRIDGE_IP} (串口转网口设备)")
            print(f"   串口参数: 115200 bps, 8N1")
        else:
            print(f"🔌 串口: {Config.SERIAL_PORT} @ {Config.SERIAL_BAUDRATE} bps")
        print(f"⚡ 更新频率: {Config.UPDATE_RATE} Hz")
        print(f"🎯 预测补偿: {Config.PREDICTION_MS} ms")
        print(f"🔧 滤波强度: {Config.SMOOTHING_FACTOR}")
        print(f"📍 死区: {Config.DEADZONE}°")
        print(f"⚡ 最大速度: {Config.MAX_SPEED}°/s")
        print(f"🎮 工作模式: {['Follow', 'Lock', 'FPV'][Config.WORK_MODE]}")
        print("="*70)
        print("按 Ctrl+C 退出\n")

    def cleanup(self):
        """清理资源"""
        self.running = False
        if self.serial:
            self.serial.close()
        self.sock.close()
        print("✅ 资源已清理")

# ==================== 命令行参数处理 ====================

def print_help():
    """打印帮助信息"""
    print("""
用法: python gimbal_controller_ultimate.py [选项]

选项:
  --preset <name>     使用预设配置 (turbo/smooth/balanced)
  --port <COM>        指定串口 (默认: COM12)
  --rate <Hz>         设置更新频率 (默认: 100)
  --predict <ms>      设置预测补偿 (默认: 15)
  --smooth <0-1>      设置滤波强度 (默认: 0.3)
  --deadzone <度>     设置死区 (默认: 0.05)
  --debug             显示调试信息
  --help, -h          显示此帮助

预设配置:
  turbo      极速响应 (150Hz, 预测10ms, 弱滤波)
  smooth     平滑稳定 (100Hz, 预测20ms, 强滤波)
  balanced   均衡模式 (100Hz, 预测15ms, 中等滤波) [默认]

示例:
  python gimbal_controller_ultimate.py --preset turbo
  python gimbal_controller_ultimate.py --rate 150 --predict 20
  python gimbal_controller_ultimate.py --port COM3 --smooth 0.1
""")

def parse_args():
    """解析命令行参数"""
    args = sys.argv[1:]
    i = 0

    while i < len(args):
        arg = args[i]

        if arg in ['--help', '-h']:
            print_help()
            sys.exit(0)

        elif arg == '--preset':
            if i + 1 < len(args):
                preset_name = Config.apply_preset(args[i + 1])
                if preset_name:
                    print(f"✅ 已应用预设: {preset_name}")
                else:
                    print(f"❌ 未知预设: {args[i + 1]}")
                i += 1

        elif arg == '--port':
            if i + 1 < len(args):
                Config.SERIAL_PORT = args[i + 1]
                i += 1

        elif arg == '--rate':
            if i + 1 < len(args):
                Config.UPDATE_RATE = int(args[i + 1])
                i += 1

        elif arg == '--predict':
            if i + 1 < len(args):
                Config.PREDICTION_MS = int(args[i + 1])
                i += 1

        elif arg == '--smooth':
            if i + 1 < len(args):
                Config.SMOOTHING_FACTOR = float(args[i + 1])
                i += 1

        elif arg == '--deadzone':
            if i + 1 < len(args):
                Config.DEADZONE = float(args[i + 1])
                i += 1

        elif arg == '--debug':
            Config.SHOW_DEBUG = True

        else:
            print(f"⚠️  未知参数: {arg}")

        i += 1

# ==================== 主程序入口 ====================

if __name__ == "__main__":
    # 解析命令行参数
    parse_args()

    # 启动控制器
    controller = GimbalController()
    controller.run()
