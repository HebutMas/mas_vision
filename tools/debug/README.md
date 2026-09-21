# 远程调试(Rerun)

调试是**编译期可选**的:默认关闭,关闭时不链接任何调试依赖,零开销。

## 目录内容

| 文件 | 作用 |
|---|---|
| `debug.hpp` | 唯一对外接口。`tools::debug::Sink`,header-only。 |
| `test.cpp` | 链路测试,只在调试构建时生成 `debug_test`,并由 ctest 运行。 |

第三方依赖在仓库根的 `3rdparty/`。

## 工作方式

```
机器人(无头)                                  开发机
  apps/infantry                                  rerun --serve-web
      │ tools::debug::Sink                           ▲
      │   image / data / log                          │ gRPC :9876
      └────────── connect_grpc ───────────────────────┘
                                                         浏览器 http://<开发机IP>:9090
```

数据走 gRPC,图像在机器人侧压成 JPEG 再发,所以一条千兆网线足够跑满相机帧率。

> 已知限制:如果 Viewer 中途挂掉,底层 gRPC 客户端可能阻塞在进程退出阶段。调试时请保证
> Viewer 全程在线;真遇到卡住,重启机器人进程即可。

## api接口

```cpp
#include "tools/debug/debug.hpp"

tools::debug::Sink debug("rm_vision.infantry", "rerun+http://<开发机IP>:9876/proxy");
```

| 接口 | 说明 |
|---|---|
| `image(path, bgr [, jpeg_quality])` | 发送一张 OpenCV BGR 图像,内部 JPEG 压缩。默认质量 80。 |
| `data(path, value)` | 发送单个标量,Viewer 中是一条曲线。 |
| `data(path, values, names)` | 发送多分量,第 i 个分量记到 `<path>/<names[i]>`。 |
| `log(level, message [, path])` | 发送带等级的文本日志,`Level` 有 `debug/info/warn/error`。 |

主循环里典型用法:

```cpp
debug.set_frame(frame);                                       // 帧序号时间轴,可逐帧回放
debug.image("camera/image", bgr);                             // 图像
debug.data("ekf/yaw", target.yaw);                            // 单值曲线
debug.data("imu/quat", {imu.w, imu.x, imu.y, imu.z},
           {"w", "x", "y", "z"});                             // → imu/quat/w, imu/quat/x ...
debug.log(tools::debug::Level::warn, "target lost");          // 带等级日志
```

- `set_frame` 建议在主循环开头调用一次。Viewer 里可以拖时间轴逐帧回放,配合图像和曲线对比。
- 多分量接口需要自己写分量名,`values` 与 `names` 必须等长,否则整次调用被跳过。
  结构体拆开写即可:

  ```cpp
  debug.data("imu/quat", {imu.w, imu.x, imu.y, imu.z}, {"w", "x", "y", "z"});
  ```

- 路径推荐按模块分层,例如 `camera/image`、`det/boxes`、`ekf/yaw`、`shoot/decision`。
- 未定义 `RM_DEBUG` 时以上调用全部是空操作,参数甚至不会被求值之外地使用。


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

`RM_DEBUG` 打开后,首次配置/构建会:

1. 解压 `3rdparty/rerun_cpp_sdk.zip`(约 56 MB,含各平台预编译 `rerun_c`);
2. 本机构建 Apache Arrow 18.0.0 静态库(源码取自 `3rdparty/apache-arrow-18.0.0.tar.gz`,
   其构建过程所需的 xsimd 取自 `3rdparty/xsimd-13.0.0.tar.gz`)。
   首次约 5~10 分钟,之后增量构建很快。

可覆盖的 CMake 变量:

| 变量 | 默认值 | 说明 |
|---|---|---|
| `RM_RERUN_SDK_ZIP` | `3rdparty/rerun_cpp_sdk.zip` | SDK zip 的路径或 URL |
| `RM_ARROW_TARBALL` | `3rdparty/apache-arrow-18.0.0.tar.gz` | Arrow 源码包路径 |
| `RM_ARROW_URL` | 华为云镜像 | 本地 Arrow 包缺失时的下载地址 |
| `RM_ARROW_MD5` | `96a4e40287137867c9fe7a2b6c3e5083` | Arrow 源码包校验和 |
| `RM_XSIMD_TARBALL` | `3rdparty/xsimd-13.0.0.tar.gz` | Arrow 构建所需的 xsimd 源码包路径 |
| `RM_XSIMD_SHA256` | `8bdbbad0...41110a3` | xsimd 源码包校验和 |

> 预置失败不影响构建:SDK 会自动回退到它自带的 GitHub 地址,只是会很慢。

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

**首次构建卡在 Arrow / xsimd**
看它的下载进度即可;两者都已随仓库提供源码包,正常不会走网络。若确实卡住,可用
`-DRM_ARROW_URL` 换个镜像,或手工把源码包放到 `-DRM_ARROW_TARBALL`、`-DRM_XSIMD_TARBALL`
指向的位置。

**Viewer 打开后一片空白**
确认机器人进程真的在发数据(`Sink::active()` 为 true),以及浏览器连的是同一台 Viewer。
