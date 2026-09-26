# rm_vision

RoboMaster 自瞄视觉框架。纯 C++17,无 ROS 依赖,面向无头 Linux 部署。

## 目标平台

- Ubuntu 22.04 LTS
- Debian 13 (trixie)

开发机可以是任意 Linux(Fedora / Arch 等),但部署与验收以以上两个发行版为准。

## 架构设计

四层单向依赖,`apps -> modules -> hardware -> tools`:


## 目录结构

```
.
├── CMakeLists.txt
├── 3rdparty/                    # 随仓库携带的第三方依赖(MVS SDK、Rerun/Arrow 等)
├── docs/
│   └── build.md                 # 开发环境与构建文档
├── docker/                      # 发行版(Ubuntu 22.04 / Debian 13)开发镜像
├── scripts/
│   ├── build_opencv.sh          # 构建 OpenCV 4.10(Ubuntu / Debian / Fedora)
│   └── lint.sh                  # 格式化 + clang-tidy + cppcheck
├── tools/                       # 工具层
│   ├── config/                  # config,YAML 配置解析(yaml-cpp)
│   ├── time/                    # time,统一时间基准
│   ├── latest_frame/            # latest_frame
│   ├── exiter/                  # exiter,SIGINT/SIGTERM 退出标志
│   └── debug/                   # debug,远程调试
├── hardware/                    # 硬件层
│   └── hikcamera/               # hikcamera,海康 USB3.0 相机驱动
└── apps/                        # 应用层
    └── templates/               # 新兵种示例
```

## 快速开始

构建依赖见 [`docs/build.md`](docs/build.md)。

编译哪些兵种由根 `CMakeLists.txt` 中的 `set(APPS ...)` 决定,改哪个编哪个:

```cmake
# Options: infantry hero sentry dart
set(APPS infantry)
```

```bash
cmake -S . -B build -G Ninja -DCMAKE_BUILD_TYPE=Release
cmake --build build -j4
ctest --test-dir build --output-on-failure   # ctest 测试
./build/apps/infantry
```

## 扩展指南

### 新增一个兵种

从 `apps/templates/` 复制为 `apps/<new>/`,按该兵种下位机协议实现报文编解码与配置,然后把名字加入根 `CMakeLists.txt` 的 `set(APPS ...)` 与 `apps/CMakeLists.txt` 的 `KNOWN_APPS`。可参考 `apps/infantry/`。

## 远程调试
通过网线把图像、检测框、跟踪状态和决策量实时推送到开发机的[Rerun](https://rerun.io) Viewer。

## 代码规范

- `.clang-format`:格式化规则。
- `.clang-tidy`:基于 Clang AST 的静态检查。
- `.cppcheck` / `.cppcheck-suppressions`:cppcheck 独立解析的缺陷检查配置。

```bash
./scripts/lint.sh                        # 一次跑完格式 + clang-tidy + cppcheck
find tools apps \( -name '*.hpp' -o -name '*.cpp' \) -print0 | xargs -0 clang-format -i
run-clang-tidy -p build -quiet
cppcheck --project=build/compile_commands.json
```

## 配置格式

当前为扁平 `key: value`,以 `#` 开头为注释。示例:

```yaml
camera.type: null
transport.type: null
detector.algorithm: null
track.algorithm: null
shoot.algorithm: null
```

## 参考项目

- [`sp_vision_25`](https://github.com/TongjiSuperPower/sp_vision_25)
- [`julyfun/rm.cv.fans`](https://github.com/julyfun/rm.cv.fans)
- [`chenjunnn/rm_vision`](https://github.com/chenjunnn/rm_vision)
