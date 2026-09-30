cmake_minimum_required(VERSION 3.16)
project(rm_vision LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# 开启警告
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra)
endif()

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

# Which apps to build; edit this list. Options: infantry hero sentry dart
set(APPS infantry)

# 远程调试(Rerun):默认开启。可在配置时用 -DRM_DEBUG=OFF 关闭,跳过 Rerun/Arrow 编译。
option(RM_DEBUG "Enable Rerun remote-debugging (builds Arrow, slow)" ON)

find_package(OpenCV REQUIRED)
find_package(yaml-cpp REQUIRED)

# yaml-cpp 在 Ubuntu 22.04(0.7.0)只导出 yaml-cpp,在 Debian 13(0.8.0)才带命名空间,
# 两边的 imported target 名不同,这里统一成一个变量供下层链接。
if(TARGET yaml-cpp::yaml-cpp)
  set(RM_YAML_CPP_TARGET yaml-cpp::yaml-cpp)
else()
  set(RM_YAML_CPP_TARGET yaml-cpp)
endif()

# Eigen
find_package(Eigen3 REQUIRED)

include_directories(${PROJECT_SOURCE_DIR})

enable_testing()

if(RM_DEBUG)
  # Rerun C++ SDK:需先按 docs/build.md 第四节编译安装到系统(/usr/local),
  find_package(rerun_sdk REQUIRED)
endif()

add_subdirectory(tools)
add_subdirectory(hardware)
add_subdirectory(modules)
add_subdirectory(apps)
