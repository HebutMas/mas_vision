add_library(hardware INTERFACE)

# 海康相机驱动依赖厂商 MVS SDK。先按 docs/build.md 安装,默认 /opt/MVS;
# 装在别处用 -DMVCAM_SDK_PATH=<SDK 根目录> 指定(优先级最高,其次同名环境变量)。
if(DEFINED ENV{MVCAM_SDK_PATH} AND NOT DEFINED MVCAM_SDK_PATH)
  set(MVCAM_SDK_PATH "$ENV{MVCAM_SDK_PATH}" CACHE PATH "HikRobot MVS SDK root")
else()
  set(MVCAM_SDK_PATH "/opt/MVS" CACHE PATH "HikRobot MVS SDK root")
endif()

# SDK 库目录按架构命名:x86_64 为 lib/64,ARM 为 lib/aarch64。
if(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64|arm64")
  set(_mvs_arch aarch64)
else()
  set(_mvs_arch 64)
endif()
set(MVS_LIBRARY_DIRS "${MVCAM_SDK_PATH}/lib/${_mvs_arch}")

find_path(MVS_INCLUDE_DIR NAMES MvCameraControl.h
  PATHS "${MVCAM_SDK_PATH}/include" NO_DEFAULT_PATH)
find_library(MVS_LIBRARIES NAMES MvCameraControl
  PATHS "${MVS_LIBRARY_DIRS}" NO_DEFAULT_PATH)
if(NOT MVS_INCLUDE_DIR OR NOT MVS_LIBRARIES)
  message(FATAL_ERROR "未找到 MVS SDK:请先按 docs/build.md 安装(默认 /opt/MVS),"
                      "或用 -DMVCAM_SDK_PATH=<SDK 根目录> 指定")
endif()

add_library(hikcamera STATIC hikcamera/hikcamera.cpp)
target_include_directories(hikcamera PUBLIC ${MVS_INCLUDE_DIR})
target_link_libraries(hikcamera PUBLIC ${OpenCV_LIBS} ${MVS_LIBRARIES})
target_link_libraries(hardware INTERFACE hikcamera)
# SDK 动态库不在系统默认搜索路径时,给可执行文件写死 rpath 保证运行时找到。
target_link_options(hikcamera PUBLIC "-Wl,-rpath,${MVS_LIBRARY_DIRS}")

# 测试:无相机时失败,有相机时抓一帧;RM_DEBUG 下顺带把画面送 Rerun。
add_executable(hikcamera_test hikcamera/test.cpp)
target_link_libraries(hikcamera_test PRIVATE hardware tools)

add_test(NAME hikcamera COMMAND hikcamera_test)
# 无相机 / 无 Viewer 时测试直接结束;RM_DEBUG 且 Viewer 在线时会连续发送 5s
set_tests_properties(hikcamera PROPERTIES TIMEOUT 60)
