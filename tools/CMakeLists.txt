set(RM_TOOLS_SOURCES
  algorithm/math_tools.cpp
  algorithm/extended_kalman_filter.cpp
  calibration/calibration.cpp
  debug/record/player.cpp)

if(RM_DEBUG)
  list(APPEND RM_TOOLS_SOURCES
    debug/video/video_encoder.cpp
    debug/record/record.cpp)
endif()

add_library(tools STATIC ${RM_TOOLS_SOURCES})
target_include_directories(tools PUBLIC ${PROJECT_SOURCE_DIR})

# 配置解析:YAML 由 yaml-cpp 提供
target_link_libraries(tools PUBLIC ${RM_YAML_CPP_TARGET})

# 链接Eigen。
target_link_libraries(tools PUBLIC Eigen3::Eigen)

# 链接OpenCV。
target_link_libraries(tools PUBLIC ${OpenCV_LIBS})

if(RM_DEBUG)
  target_compile_definitions(tools PUBLIC RM_DEBUG)
  target_link_libraries(tools PUBLIC rerun_sdk)

  # 视频调试/录制:VAAPI 编码走 libavcodec/libva。
  find_package(PkgConfig REQUIRED)
  pkg_check_modules(RM_FFMPEG REQUIRED IMPORTED_TARGET libavcodec libavutil libva)
  target_link_libraries(tools PUBLIC PkgConfig::RM_FFMPEG)
endif()

add_executable(config_test config/test.cpp)
target_link_libraries(config_test PRIVATE tools)

add_test(NAME config COMMAND config_test)

# 统一时间基准:所有硬件共用同一单调时钟,测试其稳定性 / 单调性。
add_executable(time_test time/test.cpp)
target_link_libraries(time_test PRIVATE tools)

add_test(NAME time COMMAND time_test)

# 最新帧槽:验证覆盖 / 阻塞唤醒 / close 语义。
add_executable(latest_frame_test latest_frame/test.cpp)
target_link_libraries(latest_frame_test PRIVATE tools)

add_test(NAME latest_frame COMMAND latest_frame_test)

# 算法:CRC16 测试向量 + 四元数插值缓冲 + math_tools + EKF。
add_executable(algorithm_test algorithm/test.cpp)
target_link_libraries(algorithm_test PRIVATE tools)

add_test(NAME algorithm COMMAND algorithm_test)

# 标定:棋盘格内参 + 手眼外参
add_executable(calibration_test calibration/test.cpp)
target_link_libraries(calibration_test PRIVATE tools)

add_test(NAME calibration COMMAND calibration_test)

# 退出信号:验证 SIGINT 置位退出标志。
add_executable(exiter_test exiter/test.cpp)
target_link_libraries(exiter_test PRIVATE tools)

add_test(NAME exiter COMMAND exiter_test)

if(RM_DEBUG)
  # 远程调试链路的测试:Viewer 在线/离线都必须通过,由 ctest 运行。
  add_executable(debug_test debug/test.cpp)
  target_link_libraries(debug_test PRIVATE tools)

  add_test(NAME debug_sink COMMAND debug_test)
  # 极端情况下(9876 上挂着一个不响应 gRPC 的监听)底层客户端可能卡在退出阶段,给测试设个上限,避免 ctest 永久挂住。
  set_tests_properties(debug_sink PROPERTIES TIMEOUT 30)

  # 编码器自检
  add_executable(video_test debug/video/test.cpp)
  target_link_libraries(video_test PRIVATE tools)
  add_test(NAME video COMMAND video_test)
  set_tests_properties(video PROPERTIES TIMEOUT 60)

  # 录制
  add_executable(record_test debug/record/test.cpp)
  target_link_libraries(record_test PRIVATE tools)
  add_test(NAME record COMMAND record_test)
  set_tests_properties(record PROPERTIES TIMEOUT 60)
endif()
