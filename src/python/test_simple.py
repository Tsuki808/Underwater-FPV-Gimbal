#!/usr/bin/env python3
"""最简单的云台测试 - 发送固定角度"""
import socket
import struct
import time

# CRC16计算
CRC_TABLE = [0x0000, 0x1021, 0x2042, 0x3063, 0x4084, 0x50a5, 0x60c6, 0x70e7,
             0x8108, 0x9129, 0xa14a, 0xb16b, 0xc18c, 0xd1ad, 0xe1ce, 0xf1ef]

def calc_crc16(data):
    crc = 0
    for byte in data:
        da = (crc >> 12) & 0x0F
        crc = ((crc << 4) & 0xFFFF) ^ CRC_TABLE[da ^ (byte >> 4)]
        da = (crc >> 12) & 0x0F
        crc = ((crc << 4) & 0xFFFF) ^ CRC_TABLE[da ^ (byte & 0x0F)]
    return crc

def create_packet(cmd, pitch, yaw, roll, work_mode=0):
    p = bytearray(40)
    p[0:2] = b'\xA9\x5B'
    p[2] = (0b000 << 5) | cmd
    p[3] = 0x00
    
    control_byte = (0b000 << 5) | (0b0 << 4) | (work_mode << 2) | 0b00
    p[4] = control_byte
    struct.pack_into('<h', p, 5, int(roll * 100))
    p[7] = control_byte
    struct.pack_into('<h', p, 8, int(-pitch * 100))  # Pitch取反
    p[10] = control_byte
    struct.pack_into('<h', p, 11, int(yaw * 100))
    
    crc = calc_crc16(p[:38])
    struct.pack_into('>H', p, 38, crc)
    return p

# 连接
sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
sock.connect(('192.168.1.50', 4019))
print("✅ 已连接到 192.168.1.50:4019\n")

# 测试Follow模式 (work_mode=0)
print("="*70)
print("测试 Follow 模式")
print("="*70)

# 启动命令
startup = create_packet(cmd=2, pitch=0, yaw=0, roll=0, work_mode=0)
print(f"\n>>> 发送启动命令 (cmd=2)")
print(f"    数据包: {startup[:13].hex(' ').upper()} ... {startup[38:40].hex(' ').upper()}")
sock.send(startup)
time.sleep(1)

# 测试角度
tests = [
    (0, 0, 0, "归中"),
    (10, 0, 0, "Pitch +10°"),
    (0, 0, 0, "归中"),
    (-10, 0, 0, "Pitch -10°"),
    (0, 0, 0, "归中"),
    (0, 10, 0, "Yaw +10°"),
    (0, 0, 0, "归中"),
    (0, -10, 0, "Yaw -10°"),
    (0, 0, 0, "归中"),
]

print(f"\n>>> 发送控制命令:")
for pitch, yaw, roll, desc in tests:
    packet = create_packet(cmd=4, pitch=pitch, yaw=yaw, roll=roll, work_mode=0)
    sock.send(packet)
    print(f"  发送: {desc:15s} | P={pitch:+4.0f}° Y={yaw:+4.0f}° R={roll:+4.0f}°")
    if pitch != 0 or yaw != 0:
        print(f"    数据包: {packet[:13].hex(' ').upper()}")
    time.sleep(2)  # 每个动作保持2秒

print("\n✅ 测试完成")
sock.close()

