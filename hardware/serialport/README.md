# serialport

## 帧格式

固定长度,无长度字节:`[header][payload sizeof(ReceivePacket/SendPacket)][CRC16 小端]`。

```
收:  0x5A | ReceivePacket | CRC16_lo | CRC16_hi
发:  0xA5 | SendPacket    | CRC16_lo | CRC16_hi
```

- **CRC16**:poly `0x8408`,init `0xFFFF`,输入/输出反转,xorout `0x0000`;与下位机 `crc_rm.c` 一致;校验范围为 header + payload。
- **帧头**区分方向:电控 → 视觉为 `0x5A`,视觉 → 电控为 `0xA5`。

帧长公式:`FRAME_RX_SIZE = 1 + sizeof(ReceivePacket) + 2`,`FRAME_TX_SIZE = 1 + sizeof(SendPacket) + 2`。


## 使用

### 协议

收发结构体写在 `hardware/serialport/serialtypes.hpp`,协议变更只改这里:

```cpp
// 电控 -> 视觉(收)。
struct ReceivePacket
{
  std::uint8_t mode;    // 自瞄模式
  float qw, qx, qy, qz; // 云台姿态四元数
  // SerialPort 用它把每帧姿态送入 IMU 插值缓冲。
  [[nodiscard]] Eigen::Quaternionf quaternion() const { return {qw, qx, qy, qz}; }
} __attribute__((packed));

// 视觉 -> 电控(发)。
struct SendPacket
{
  float target_yaw;         // 目标云台偏航角(弧度)
  float target_pitch;       // 目标云台俯仰角(弧度)
  std::uint8_t fire_advice; // 0:不射击,1:射击
} __attribute__((packed));
```

**约定**:`ReceivePacket` 必须提供 `Eigen::Quaternionf quaternion() const`,用于 IMU 姿态缓冲。

### 在 config.yaml 里配置

```yaml
serial:
  debug: false
  port: /dev/gimbal
  baudrate: 115200
  bytesize: 8
  parity: none
  stopbits: 1
  flowcontrol: none
  timestamp_offset: 0.0
```

| 键 | 默认 | 说明 |
|---|---|---|
| `serial.debug` | `false` | 是否打印接收的 16 进制原始数据 |
| `serial.port` | `/dev/gimbal` | 串口设备路径 |
| `serial.baudrate` | `115200` | 波特率,须在支持列表内 |
| `serial.bytesize` | `8` | 数据位,5/6/7/8 |
| `serial.parity` | `none` | 校验,none/even/odd |
| `serial.stopbits` | `1` | 停止位,1/2 |
| `serial.flowcontrol` | `none` | 流控,none/rtscts |
| `serial.timestamp_offset` | `0.0` | IMU 姿态时间戳偏移(秒):到达时刻 + offset |

> 跑一次 `sudo scripts/serial_setup.sh`将ttyACMX/ttyUSBX 固定成 `/dev/gimbal`

### main 里读取配置并注册

```cpp
#include "hardware/serialport/serialport.hpp"
#include "tools/config/config.hpp"

int main()
{
  const tools::config::Config config("apps/infantry/config.yaml");
  hardware::serialport::SerialPort serial(config); // 构造函数内部读取 serial 段并打开串口
}
```

### 收发

```cpp
// 发
hardware::serialport::SendPacket command{};
serial.send(command);

// 收
hardware::serialport::ReceivePacket state;
if (serial.frames().try_pop(state)) { /* 用电控状态 */ }

// 用相机帧的采集时刻取该时刻的云台姿态。
hardware::hikcamera::HikFrame frame;
if (camera.frames().wait_for(frame, std::chrono::milliseconds(100)))
{
  const Eigen::Quaternionf q = serial.quaternion_at(frame.timestamp);
}
```

## API

| 成员 | 说明 |
|---|---|
| `SerialPort(const tools::config::Config &)` | 读取配置 `serial` 段并打开串口 |
| `is_open()` | 串口当前是否已打开(重连期间为 `false`) |
| `send(const SendPacket &)` | 组帧发送 |
| `frames()` | `tools::LatestFrame<ReceivePacket> &`,消费者用 `wait` / `wait_for` / `try_pop` 取最新帧 |
| `quaternion_at(time)` | 取 `time` 时刻的云台姿态(`Eigen::Quaternionf`) |

## 掉线重连

串口偶发断开时,接收线程按 5s 间隔持续重连,直到成功


## 构建与测试

`hardware/CMakeLists.txt` 把 `serialport.cpp` 编成静态库 `serialport`,并入 `hardware`:

```cmake
add_library(serialport STATIC serialport/serialport.cpp)
target_link_libraries(serialport PUBLIC tools)
target_link_libraries(hardware INTERFACE serialport)
```

测试 `hardware/serialport/test.cpp` 先跑**伪终端(`openpty`)回环**(组帧 / 解析 / 重同步 / 掉线检测 / 设备缺失),再跑**真实串口**(默认 `/dev/ttyACM0`,`RM_SERIAL_PORT` 可覆盖;读帧率 + IMU 姿态 + 试发;无设备自动跳过):

```bash
cmake --build build -j4
ctest --test-dir build -R serialport --output-on-failure
```
