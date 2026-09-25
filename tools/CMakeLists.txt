# debug 是编译期可选件:只有 RM_DEBUG=ON 时才链接 Rerun SDK 并构建其测试。
add_library(tools INTERFACE)
target_include_directories(tools INTERFACE ${PROJECT_SOURCE_DIR})

# 配置解析:YAML 由 yaml-cpp 提供
target_link_libraries(tools INTERFACE ${RM_YAML_CPP_TARGET})

add_executable(config_test config/test.cpp)
target_link_libraries(config_test PRIVATE tools)

add_test(NAME config COMMAND config_test)

if(RM_DEBUG)
  target_compile_definitions(tools INTERFACE RM_DEBUG)
  target_link_libraries(tools INTERFACE rerun_sdk)

  # 远程调试链路的冒烟测试:Viewer 在线/离线都必须通过,由 ctest 运行。
  add_executable(debug_test debug/test.cpp)
  target_link_libraries(debug_test PRIVATE tools ${OpenCV_LIBS})

  add_test(NAME debug_sink COMMAND debug_test)
  # 兜底:极端情况下(9876 上挂着一个不响应 gRPC 的监听)底层客户端可能卡在退出阶段,
  # 给测试设个上限,避免 ctest 永久挂住。
  set_tests_properties(debug_sink PROPERTIES TIMEOUT 30)
endif()
