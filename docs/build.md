# 开发环境与构建

## 依赖总览

| 依赖 | 版本要求 | 说明 |
|---|---|---|
| CMake | >= 3.16 | apt 安装即可 |
| Ninja | 任意 | apt 安装即可  |
| GCC / G++ | >= 11 | apt 安装即可  |
| OpenCV | = 4.10.0 | 使用 scripts/build_opencv.sh 脚本安装  |

## 安装构建工具
```bash
sudo apt-get update
sudo apt-get install -y \
  build-essential \
  cmake \
  ninja-build \
  git \
  pkg-config
```

## 从源码构建 OpenCV 4.10
> 仓库 `scripts/` 下提供 `build_opencv.sh`,会自动安装编译依赖、下载并校验源码与第三方依赖(ippicv / ade)、关闭 CUDA、启用 IPP/TBB/V4L/GStreamer,最后安装到指定前缀。
```bash
./scripts/build_opencv.sh -j4 --prefix /usr/local --register-ldconfig
```

常用参数:

| 参数 | 说明 |
|---|---|
| `-j N` | 并行编译任务数 |
| `--prefix DIR` | 安装前缀,默认 `/usr/local` |
| `--workdir DIR` | 源码 / 构建 / 缓存目录,默认 `~/opencv-build` |
| `--register-ldconfig` | 写入 `/etc/ld.so.conf.d/opencv-4.10.conf` 并刷新动态库缓存 |
| `--gitee` | 使用 Gitee 镜像克隆源码(网络受限时使用) |

说明:
- 默认从多个 GitHub 镜像依次尝试下载源码,并对每个文件做 MD5 校验;可通过环境变量 `OPENCV_MIRROR` 指定单一镜像前缀。
- 脚本内已设置 `CMAKE_POLICY_VERSION_MINIMUM=3.5`,以兼容 CMake 4.x。
- 若不使用 `--register-ldconfig`,需保证运行时能找到 `/usr/local/lib64`(或 `/usr/local/lib`),否则可能出现找不到 `libopencv_*.so` 的问题。

验证:

```bash
pkg-config --modversion opencv4
```

## 构建项目

```bash
cmake -S . -B build -G Ninja -DCMAKE_BUILD_TYPE=Release
cmake --build build -j4
```
编译后可执行文件路径 `build/apps/`:

需要编译哪个兵种,在根 `CMakeLists.txt` 中修改:

```cmake
# Options: infantry hero sentry dart
set(APPS infantry)
```

支持多个:`set(APPS infantry sentry)`。

## 代码规范
格式与静态检查配置在仓库根目录:`.clang-format` / `.clang-tidy` / `.cppcheck`(需 clang-format、clang-tidy、cppcheck,建议 clang 18 及以上)。

```bash
# 一键运行全部检查(格式化 + clang-tidy + cppcheck)
./scripts/lint.sh

# 或分别执行
find apps \( -name '*.hpp' -o -name '*.cpp' \) -print0 | xargs -0 clang-format --dry-run --Werror  # 格式检查
run-clang-tidy -p build -quiet          # Clang AST 语义检查
cppcheck --project=build/compile_commands.json  # 独立解析,缺陷类检查
```

说明:

- `clang-tidy` 与 `cppcheck` 需要先生成编译数据库:`cmake -S . -B build -G Ninja`。
- `cppcheck` 会自动读取仓库根目录的 `.cppcheck` 配置文件(检查等级、退出码、C++ 标准等),无需重复传参。
- 抑制规则写在 `.cppcheck-suppressions`;源码内也可用 `// cppcheck-suppress <id>` 就近抑制。
- 两者互补:cppcheck 独立解析、快,擅长数据流/未初始化/越界等缺陷;clang-tidy 基于 Clang AST,语义准确、可自动修复。重复告警各自抑制即可。
- 编辑器若支持 clangd,会自动读取 `.clang-format` / `.clang-tidy`。

## 运行

每个兵种一个可执行文件,默认读取自身目录下的 `config.yaml`,也可显式传入路径:

```bash
./build/apps/infantry apps/infantry/config.yaml
```

## 部署
注册开机自启。以 systemd 服务:

```ini
# /etc/systemd/system/rm-vision.service
[Unit]
Description=RM Vision
After=network.target

[Service]
Type=simple
User=mas # 运行用户
WorkingDirectory=/home/rm/rm_vision #代码路径
ExecStart=/home/rm/rm_vision/build/apps/infantry /home/rm/rm_vision/apps/infantry/config.yaml #可执行文件路径
Restart=on-failure #失败时重启
RestartSec=1 #重启间隔

[Install]
WantedBy=multi-user.target #开机自启
```

```bash
sudo systemctl daemon-reload
sudo systemctl enable --now rm-vision
journalctl -u rm-vision -f
```

## 常见问题

- **CMake 找不到 OpenCV**:确认 `pkg-config --modversion opencv4` 有输出;若安装在 `/usr/local`,用 `-DOpenCV_DIR=...` 指定。
- **运行时找不到 `libopencv_core.so.4.10`**:使用 `scripts/build_opencv.sh --register-ldconfig`,或手动把库目录加入 `/etc/ld.so.conf.d/` 后执行 `sudo ldconfig`。
- **CMake 4.x 报 `cmake_minimum_required` 兼容性错误**:仅影响从源码构建 OpenCV 的场景,脚本已处理;本项目自身 `cmake_minimum_required(VERSION 3.16)` 无此问题。
