set(KNOWN_APPS infantry hero sentry dart)

foreach(app ${APPS})
  if(NOT app IN_LIST KNOWN_APPS)
    message(FATAL_ERROR "Unknown app '${app}'. Known apps: ${KNOWN_APPS}")
  endif()

  add_executable(${app} ${app}/main.cpp)
  target_link_libraries(${app} PRIVATE tools ${OpenCV_LIBS})

  if(RM_DEBUG)
    # 无 Viewer 时 ${app} 会立即跳过调试输出并返回 0;超时兜底防止退出阶段卡住。
    add_test(NAME ${app}_debug COMMAND ${app})
    set_tests_properties(${app}_debug PROPERTIES TIMEOUT 60)
  endif()
endforeach()

message(STATUS "Building apps: ${APPS}")
