# rm_vision

RoboMaster 自瞄视觉框架。纯 C++17,无 ROS 依赖,面向无头 Linux 部署。

## 目标平台

- Ubuntu 22.04 LTS
- Debian 13 (trixie)

开发机可以是任意 Linux(Fedora / Arch 等),但部署与验收以以上两个发行版为准。

## 架构设计

四层单向依赖,`apps -> modules -> hardware -> tools`:

| 层 | 职责 |
|---|---|
| `apps` | 应用层(composition root):每个兵种一个可执行文件,单线程编排取图 → 检测 → 跟踪 → 决策 → 发送,并负责本兵种的配置与报文协议 |
| `modules` | 算法层:`detector`(装甲板检测)/ `track`(跟踪估计)/ `shoot`(瞄准开火决策) |
| `hardware` | 硬件层:`camera`(取图)/ `transport`(原始字节收发)/ `message`(命令与回传) |
| `tools` | 工具层:`config` / `time` / `queue` / `exiter` 等无业务依赖的基础件 |

项目采用**自顶向下**构建:先落地 `apps` 层并明确它需要的接口,再按需求逐层实现下层。

## 目录结构

```
.
├── CMakeLists.txt
├── build_opencv.sh              # 从源码构建 OpenCV 4.10(Ubuntu / Debian / Fedora)
├── docs/
│   └── build.md                 # 开发环境与构建文档
├── scripts/
│   ├── build_opencv.sh          # 从源码构建 OpenCV 4.10(Ubuntu / Debian / Fedora)
│   └── lint.sh                  # 格式化 + clang-tidy + cppcheck
└── apps/                        # 应用层
    ├── infantry/{main.cpp, config.yaml}   # 当前唯一示例实现
    └── templates/               # 新兵种脚手架(hero/sentry/dart 待按此创建)
```

## 快速开始

构建依赖与 OpenCV 的准备见 [`docs/build.md`](docs/build.md)。

编译哪些兵种由根 `CMakeLists.txt` 中的 `set(APPS ...)` 决定,改哪个编哪个:

```cmake
# Options: infantry hero sentry dart
set(APPS infantry)
```

```bash
cmake -S . -B build -G Ninja -DCMAKE_BUILD_TYPE=Release
cmake --build build -j4
./build/apps/infantry
```

当前为应用层骨架:`main()` 只保留顶层流程注释,可正常编译运行;下层就位后成为常驻主循环。

## 扩展指南

### 新增一个兵种

从 `apps/templates/` 复制为 `apps/<new>/`,按该兵种下位机协议实现报文编解码与配置,然后把名字加入根 `CMakeLists.txt` 的 `set(APPS ...)` 与 `apps/CMakeLists.txt` 的 `KNOWN_APPS`。可参考 `apps/infantry/`。

## 代码规范

- `.clang-format`:格式化规则。
- `.clang-tidy`:基于 Clang AST 的静态检查。
- `.cppcheck` / `.cppcheck-suppressions`:cppcheck 独立解析的缺陷检查配置。

```bash
./scripts/lint.sh                        # 一次跑完格式 + clang-tidy + cppcheck
find apps \( -name '*.hpp' -o -name '*.cpp' \) -print0 | xargs -0 clang-format -i
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
