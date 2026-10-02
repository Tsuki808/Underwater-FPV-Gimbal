# FPV Gimbal Control Middleware

面向机器人的 FPV（First Person View，第一人称视角）云台控制中间件。

本项目实现了一条 **VR 头显姿态 → 双轴云台** 的低延迟控制链路：由 OpenTrack 输出头部三轴姿态数据（UDP），中间件解析后封装为云台私有协议指令包，经串口服务器（TCP）下发至云台执行机构，使摄像头视角实时跟随操作员视线。

---

## 1. 系统架构

```
┌──────────────┐   UDP :4242    ┌─────────────────────┐   TCP :4019   ┌──────────────┐
│  VR 头显     │  (6×double)    │   控制中间件         │  (40 字节)     │  串口服务器   │  RS485/TTL
│  OpenTrack   │ ─────────────► │  UDP 接收 / 解包     │ ────────────► │  串口转网口   │ ─────────► 云台
│  头部姿态    │   48 字节定长   │  去重·滤波·协议封装   │   私有协议帧   │  设备         │
└──────────────┘                └─────────────────────┘               └──────────────┘
```

- **输入链路**：OpenTrack 通过 UDP 向本机 `4242` 端口推送头部姿态，数据包固定 `48` 字节，含 6 个 `double`（小端序）。程序仅取后三个字段：`Yaw`（偏移 24）、`Pitch`（偏移 32）、`Roll`（偏移 40）。
- **输出链路**：程序与串口服务器建立 TCP 连接（默认 `192.168.1.50:4019`），将 `40` 字节定长二进制帧透传至下位机串口，波特率 `115200 8N1`。
- **控制频率**：C++ 版本由 UDP 收包驱动（受 OpenTrack 发包频率限制，通常 60–100 Hz）；Python 优化版提供 `100–150 Hz` 可配置发送频率。

---

## 2. 仓库结构

```
.
├── README.md
├── src/
│   ├── cpp/
│   │   ├── ultra_low_latency.cpp      # 主控制中间件（WinSock2 原生实现）
│   │   ├── udp_test.cpp               # 串口服务器端口探测工具
│   │   └── CMakeLists.txt             # CMake 构建脚本
│   └── python/
│       ├── gimbal_controller_ultimate.py  # 功能完整版：滤波/预测/预设/性能监控
│       ├── gimbal_controller.py           # 基础版：协议实现 + 串口直连
│       └── test_simple.py                 # 最小验证脚本：发送固定角度
├── docs/
│   ├── protocol.md                    # 云台私有协议指令帧解析
│   ├── design.md                      # 控制中间件设计说明
│   └── troubleshooting.md             # 联调问题排查记录
```

---

## 3. 核心模块

### 3.1 协议封装（`calculate_crc16` / `PacketBuilder`）

依据云台私有协议 V1.0.2 构造定长 40 字节指令帧。关键实现要点：

- **角度量化**：浮点角度 × 100 转为 `int16`，即控制分辨率 `0.01°`。
- **字节序**：除 CRC 外全部为**小端序**；CRC 字段为协议中唯一使用**大端序**的字段。
- **CRC-16/X-25**：采用与厂商 C 代码一致的高半字节查表法（16 项预计算表），校验范围为前 38 字节。
- **Pitch 反向修正**：因云台安装姿态，Pitch 轴需取反，即 `int16_t(-pitch * 100)`。

### 3.2 数据去重与死区（`process_angles`）

- **去重**：OpenTrack 存在爆发式重复发包，Python 版对整包做哈希比对跳过重复样本；C++ 版比对 `yaw`/`pitch` 数值是否变化。
- **滤波**：`KalmanFilter` 一维卡尔曼滤波器（Python 版），抑制传感器抖动。
- **运动预测**：`MotionPredictor` 基于最近 5 帧速度外推，补偿 `PREDICTION_MS` 毫秒链路延迟。
- **死区**：仅当角度变化超过阈值时才更新指令，过滤人体无意识微颤。

### 3.3 异步发送（`AsyncSerialManager`，Python 版）

在独立线程中以队列方式发送指令，避免 I/O 阻塞主循环；队列长度上限 10，过载时丢弃新包以保证实时性。

### 3.4 工作模式与参数预设

| 模式 | `wk_mode` | 控制字节 | 说明 |
|------|-----------|----------|------|
| Follow | 0 | `0x00` | 头部跟随，响应较快 |
| Lock | 1 | `0x04` | 姿态锁定，用于定点观测 |
| FPV | 2 | `0x08` | 第一人称自由视角 |

Python 优化版内置 `turbo`（150 Hz / 预测 10 ms）、`smooth`（100 Hz / 预测 20 ms）、`balanced`（100 Hz / 预测 15 ms）三档参数预设。

---

## 4. 构建与运行

### 4.1 C++ 版本

```bash
cd src/cpp
g++ -o ultra_low_latency.exe ultra_low_latency.cpp -lws2_32 -std=c++14
```

或使用 CMake：

```bash
cd src/cpp
cmake -B build
cmake --build build
```

运行前请修改源码顶部配置区，填入你自己的串口服务器地址：

```cpp
#define SERIAL_BRIDGE_IP   "192.168.1.50"   // 串口服务器 IP
#define SERIAL_BRIDGE_PORT 4019             // TCP 端口
#define OPENTRACK_PORT     4242             // OpenTrack UDP 端口
#define WORK_MODE          0                // 0=Follow, 1=Lock, 2=FPV
```

### 4.2 Python 版本

```bash
pip install pyserial
python src/python/gimbal_controller_ultimate.py --help
```

命令行参数：

```
--preset <name>     使用预设配置 (turbo/smooth/balanced)
--port <COM>        指定串口 (默认 COM13)
--rate <Hz>         设置更新频率
--predict <ms>      设置预测补偿
--smooth <0-1>      设置滤波强度
--deadzone <度>     设置死区
--debug             显示调试信息
```

配置项集中在 `Config` 类中，`USE_TCP = True` 为串口服务器模式，`False` 为 USB-TTL 直连串口模式（此时使用 `SERIAL_PORT`）。

### 4.3 端口探测（可选）

当不确定串口服务器的 TCP 端口时可运行：

```bash
udp_test.exe 192.168.1.50 4019
```

---

## 5. 使用前提

1. **OpenTrack** 已运行，配置 `UDP over network` 输出至 `127.0.0.1:4242`。
2. 串口服务器（串口转网口模块）已配置为 **TCP Server** 模式，波特率 `115200 8N1`，且与本机网络可达。
3. 云台已上电；程序启动时会先发送 `cmd=2` 启动指令。

---

## 6. 文档索引

| 文档 | 内容 |
|------|------|
| [`docs/protocol.md`](docs/protocol.md) | 40 字节指令帧的逐字段解析、CRC 算法 |
| [`docs/design.md`](docs/design.md) | 中间件分层设计、延迟来源分析、两份实现的差异对比 |
| [`docs/troubleshooting.md`](docs/troubleshooting.md) | 联调期间的典型问题与定位结论 |

---

## 7. 说明

- 本仓库代码为项目开发过程中的实际实现，保留了原有设计思路与算法逻辑，未做功能性改写。
- 协议字段定义参照云台厂商发布的《云台私有协议 V1.0.2》文档，该文档版权归厂商所有，未随本仓库分发；如需查阅请通过厂商官方渠道获取。本文档 `docs/protocol.md` 中的字段表为依据源码实现整理的技术说明，用于阅读代码时的对照参考。
