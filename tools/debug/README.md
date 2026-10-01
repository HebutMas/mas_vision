# 远程调试(Rerun)

调试是**编译期可选**的:默认关闭,关闭时不链接任何调试依赖,零开销。

## 目录内容

| 文件 | 作用 |
|---|---|
| `debug.hpp` | 唯一对外接口。`tools::debug::Sink`,header-only。 |
| `test.cpp` | 链路测试,只在调试构建时生成 `debug_test`,并由 ctest 运行。 |

Rerun C++ SDK 与 Apache Arrow 需编译安装到系统(`/usr/local`),CMake 通过 `find_package(rerun_sdk)` 命中;
未安装则配置直接报错。

## 工作方式

```
机器人(无头)                                  开发机
  apps/infantry                                  rerun --serve-web
      │ tools::debug::Sink                           ▲
      │   data / log / video                           │ gRPC :9876
      └────────── connect_grpc ───────────────────────┘
                                                         浏览器 http://<开发机IP>:9090
```

数据走 gRPC;视频在机器人侧编码成 H.264 再发,所以一条千兆网线足够跑满相机帧率。

> 已知限制:如果 Viewer 中途挂掉,底层 gRPC 客户端可能阻塞在进程退出阶段。调试时请保证
> Viewer 全程在线;真遇到卡住,重启机器人进程即可。

## api接口

```cpp
#include "tools/debug/debug.hpp"

tools::debug::Sink debug("rm_vision.infantry", "rerun+http://<开发机IP>:9876/proxy");
```

| 接口 | 说明 |
|---|---|
| `data(path, value)` | 发送单个标量,Viewer 中是一条曲线。 |
| `data(path, values)` | 发送多分量,整体记到 `<path>` 一个 entity,Viewer 中是同一张图内的多条曲线。 |
| `video(path, data, keyframe)` | 发送一帧 H.264 视频样本(Annex B 访问单元),`keyframe` 表示 IDR。 |
| `log(level, message [, path])` | 日志:控制台 + Rerun (`Level` 有 `debug/info/warn/error`)。 |

主循环里典型用法:

```cpp
debug.set_frame(frame);                                       // 帧序号时间轴,可逐帧回放
debug.data("ekf/yaw", target.yaw);                            // 单值曲线
debug.data("imu/quat", {imu.w, imu.x, imu.y, imu.z});         // 同一张图里的 4 条曲线
debug.video("camera/image", packet, is_key);                  // H.264 视频帧
debug.log(tools::debug::Level::warn, "target lost");          // 带等级日志
```

- `set_frame` 建议在主循环开头调用一次。Viewer 里可以拖时间轴逐帧回放,配合曲线对比。
- 路径推荐按模块分层,例如 `camera/image`、`det/boxes`、`ekf/yaw`、`shoot/decision`。
- 未定义 `RM_DEBUG` 时以上调用全部是空操作,参数甚至不会被求值之外地使用。

## 日志:控制台 + Rerun 同步

`Sink::log(level, message [, path])` 默认:始终打印到控制台(`debug/info` → stdout,`warn/error` → stderr),Viewer 可用时同一条消息一并发往Rerun。


## 使用步骤

### 开发机:启动 Viewer

安装rerun

```bash
从 https://github.com/rerun-io/rerun/releases 下载 rerun-cli 二进制
```

启动:

```bash
rerun --serve-web --bind 0.0.0.0 --port 9876
```

然后浏览器打开 `http://<开发机IP>:9090`。也可以直接跑 `rerun` 起原生窗口,它默认已监听 9876。

### 二、机器人:带调试构建并运行

把根 `CMakeLists.txt` 里的 `set(RM_DEBUG OFF)` 改成 `ON`,然后正常构建:

```bash
cmake -S . -B build -G Ninja -DCMAKE_BUILD_TYPE=Release
cmake --build build -j4
```

### 三、在自己的兵种里接入

在 `apps/<兵种>/main.cpp` 里构造一个 `Sink`,主循环内按上面的方式调用即可。
当前 `apps/infantry/main.cpp` 还是空骨架,等相机/检测器就位后再接线。

## 首次构建

先把 Rerun C++ SDK 编译安装到系统(装到 `/usr/local`,之前需先构建 Apache Arrow 18.0.0,
首次约 5~10 分钟)。之后 CMake 的 `find_package(rerun_sdk)` 直接命中,详见
[`docs/build.md`](../../docs/build.md) 第四节。

未安装时 `RM_DEBUG` 配置会直接报错,不再自动下载(不留兜底)。

## 生产构建

把 `set(RM_DEBUG ON)` 改回 `OFF` 重新构建即可。此时不会解压任何依赖、不链接 Rerun,
`debug.hpp` 退化为空操作。调试调用可以一直留在代码里。

## 常见问题

**Viewer 起不来 / Viewer 里看不到机器人**
开发机侧确认 `rerun --serve-web` 正在运行且监听 `0.0.0.0:9876`;机器人侧地址用
`rerun+http://<开发机IP>:9876/proxy`。若 Viewer 打开后一片空白,确认机器人进程真的在发数据
(`Sink::active()` 为 true),以及浏览器连的是同一台 Viewer。

**开发机能 ping 通但探不到**
检查防火墙是否放行 9876(TCP)与 9090(浏览器页面)。

**首次安装 SDK 卡在 Arrow / xsimd**
安装 Rerun SDK 时其 CMake 会先下载并编译 Arrow 与 xsimd,看下载进度即可。

**Viewer 打开后一片空白**
确认机器人进程真的在发数据(`Sink::active()` 为 true),以及浏览器连的是同一台 Viewer。
