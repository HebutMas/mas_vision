# 开发环境与构建

## 依赖总览

| 依赖 | 版本要求 | 获取方式 |
|---|---|---|
| CMake | >= 3.16 | 包管理器 |
| Ninja | 任意 | 包管理器 |
| GCC / G++ | >= 11 | 包管理器 |
| pkg-config | 任意 | 包管理器 |
| yaml-cpp | >= 0.7 | 包管理器 |
| Eigen | >= 3.4 | 包管理器 |
| OpenCV | 4.10 | 源码编译 |
| OpenVINO Runtime | >= 2025.1 | [官方下载]（#openvino） |
| MVS SDK | 5.1 | [海康官网下载](#mvs) |
| Rerun SDK | 0.38.1 | [GitHub 下载](#rerun) |
| FFmpeg 开发库 | >= 4.4 | 包管理器 |

## 一、工具链与基础库

Debian / Ubuntu:

```bash
sudo apt-get update
sudo apt-get install -y \
  build-essential cmake ninja-build git pkg-config \
  libyaml-cpp-dev libeigen3-dev
```

Fedora / RHEL:

```bash
sudo dnf install -y \
  gcc-c++ cmake ninja-build git pkgconf-pkg-config \
  yaml-cpp-devel eigen3-devel
```

## 二、OpenCV 4.10

[下载源码](https://github.com/opencv/opencv/archive/refs/tags/4.10.0.zip):

安装编译依赖:

```bash
# Debian / Ubuntu
sudo apt-get install -y g++ cmake ninja-build pkg-config \
  libtbb-dev libv4l-dev libgtk-3-dev \
  libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev \
  curl tar unzip bzip2 ca-certificates
# Fedora
sudo dnf install -y gcc-c++ cmake ninja-build pkgconf-pkg-config \
  tbb-devel libv4l-devel gtk3-devel \
  gstreamer1-devel gstreamer1-plugins-base-devel curl tar unzip bzip2
```

编译安装:

```bash
mkdir -p ~/opencv-build && tar xzf opencv-4.10.0.tar.gz -C ~/opencv-build
cd ~/opencv-build/opencv-4.10.0

cmake -S . -B build -G Ninja \
  -D CMAKE_BUILD_TYPE=Release \
  -D CMAKE_INSTALL_PREFIX=/usr/local \
  -D WITH_CUDA=OFF -D WITH_OPENCL=OFF -D WITH_FFMPEG=OFF \
  -D WITH_IPP=ON -D WITH_TBB=ON -D WITH_V4L=ON -D WITH_GSTREAMER=ON \
  -D BUILD_TESTS=OFF -D BUILD_PERF_TESTS=OFF -D BUILD_EXAMPLES=OFF \
  -D BUILD_opencv_python3=OFF -D OPENCV_GENERATE_PKGCONFIG=ON

cmake --build build -j4
sudo cmake --install build

# 让运行时能找到 /usr/local/lib64
echo /usr/local/lib64 | sudo tee /etc/ld.so.conf.d/opencv-4.10.conf
sudo ldconfig
```

验证:

```bash
pkg-config --modversion opencv4   # 期望 4.10.0
```

## 三、OpenVINO Runtime
```bash
# 具体版本 / 文件名见 https://storage.openvinotoolkit.org/repositories/openvino/packages/
curl -L -o /tmp/openvino.tgz \
  https://storage.openvinotoolkit.org/repositories/openvino/packages/2026.3.1/linux/openvino_toolkit_ubuntu22_2026.3.1.22476.56d9685302d_x86_64.tgz
sudo mkdir -p /opt/openvino
sudo tar xzf /tmp/openvino.tgz -C /opt/openvino --strip-components=1
sudo chmod -R a+rX /opt/openvino
```

配置时用 `-DOpenVINO_DIR` 指向它

```bash
cmake -S . -B build -G Ninja -DCMAKE_BUILD_TYPE=Release \
  -DOpenVINO_DIR=/opt/openvino/runtime/cmake
```

验证

```bash
./build/modules/auto_armor/auto_armor_detection_test
```

## 四、海康 MVS SDK

海康相机驱动必需。从海康机器人官网下载 **MVS V5.1.0(Linux x86_64)**:

- 下载中心:https://www.hikrobotics.com/cn/machinevision/service/download/
- 在页面搜索 `MVS`,选择 `Machine Vision Software MVS V5.1.0(Linux)`,下载 x86_64(amd64)包。

安装(解压 SDK 并注册动态库):

```bash
tar xzf MVS-5.1.0_x86_64_*.tar.gz -C /tmp
cd /tmp/MVS-*
sudo ./setup.sh          # 默认装到 /opt/MVS 并注册库路径
```

## 五、Rerun SDK(远程调试)

[下载 C++ SDK 包](https://github.com/rerun-io/rerun/releases/download/0.38.1/rerun_cpp_sdk.zip)

```bash
unzip rerun_cpp_sdk.zip
cmake -S rerun_cpp_sdk -B rerun-build -G Ninja \
  -D CMAKE_BUILD_TYPE=Release -D CMAKE_INSTALL_PREFIX=/usr/local
cmake --build rerun-build -j4
sudo cmake --install rerun-build
```

> SDK 的 CMake 会自动下载并编译 Apache Arrow 18.0.0,请确保有良好的github连接

**Viewer(开发机侧,用于看数据):**

```bash
# 从 https://github.com/rerun-io/rerun/releases/tag/0.38.1 下载 CLI 二进制
rerun --serve-web --bind 0.0.0.0 --port 9876   # 浏览器打开 http://<开发机IP>:9090
```

## 六、视频调试依赖

`RM_DEBUG` 下,相机画面经 VAAPI **硬件编码**成 H.264 再由 Rerun 视频流显示。需要 ffmpeg 开发库:

```bash
# Debian / Ubuntu
sudo apt-get install -y libavcodec-dev libavutil-dev libva-dev mesa-va-drivers ffmpeg
# Fedora
sudo dnf install -y ffmpeg-devel libva-devel ffmpeg
```

## 配置与构建

```bash
cmake -S . -B build -G Ninja -DCMAKE_BUILD_TYPE=Release \
  -DOpenVINO_DIR=/opt/openvino/runtime/cmake
cmake --build build -j4
```

需要编译哪个兵种,在根 `CMakeLists.txt` 中修改:

```cmake
# Options: infantry hero sentry dart
set(APPS infantry)
```

支持多个:`set(APPS infantry sentry)`。可执行文件生成在 `build/apps/`。

常用配置开关:

| 变量 | 默认 | 说明 |
|---|---|---|
| `-DRM_DEBUG=OFF` | `ON` | 关闭远程调试 |
| `-DMVCAM_SDK_PATH=<dir>` | `/opt/MVS` | MVS SDK 安装路径 |
| `-DOpenVINO_DIR=<dir>` | 自动查找 | OpenVINO Runtime 的 CMake 配置目录(默认 `/opt/openvino/runtime/cmake`) |

## 测试

```bash
cmake --build build -j4
ctest --test-dir build --output-on-failure
```
测试在缺少硬件时自动跳过(无相机、无 `/dev/ttyACM0`、无 `/dev/dri/renderD128` 时不判失败)。

## 运行

每个兵种一个可执行文件,`config.yaml` 的路径在**编译期**由 CMake 注入为绝对路径(`RM_CONFIG_PATH`)

```bash
./build/apps/infantry
```

## 部署

### 串口设备

运行脚本,从列表里选一个设备,固定成 `/dev/gimbal` 

```bash
sudo scripts/serial_setup.sh
```

### 开机自启

以 systemd 服务:

```ini
# /etc/systemd/system/rm-vision.service
[Unit]
Description=RM Vision
After=network.target

[Service]
Type=simple
User=mas # 运行用户
WorkingDirectory=/home/rm/rm_vision # 代码路径
ExecStart=/home/rm/rm_vision/build/apps/infantry # 可执行文件路径
Restart=on-failure # 失败时重启
RestartSec=1 # 重启间隔

[Install]
WantedBy=multi-user.target # 开机自启
```

```bash
sudo systemctl daemon-reload
sudo systemctl enable --now rm-vision
journalctl -u rm-vision -f
```

## 代码规范

格式与静态检查配置在仓库根目录:`.clang-format` / `.clang-tidy` / `.cppcheck`
(需 clang-format、clang-tidy、cppcheck,建议 clang 18 及以上)。

```bash
# 一键运行全部检查(格式化 + clang-tidy + cppcheck)
./scripts/lint.sh
```

## Docker 环境(可选)

`docker/` 提供两个发行版镜像(Ubuntu 22.04 / Debian 13)

镜像需要 MVS SDK:把海康下载的压缩包放到仓库根目录、命名为 `MVS.tar.gz`
(或在构建时用 `MVS_TARBALL=<相对路径或URL>` 指定,见 `docker/compose.yaml`)。

```bash
# 构建(需已放置 MVS.tar.gz)
docker compose -f docker/compose.yaml build
# 或显式指定 MVS 包位置/URL
MVS_TARBALL=/path/to/MVS-5.1.0_x86_64.tar.gz docker compose -f docker/compose.yaml build

# 开发(仓库挂载在 /work)
docker compose -f docker/compose.yaml run -T --rm ubuntu22 \
  bash -c 'cmake -S . -B build-docker -G Ninja && cmake --build build-docker -j && ctest --test-dir build-docker --output-on-failure'
```
