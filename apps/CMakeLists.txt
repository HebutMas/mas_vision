set(KNOWN_APPS infantry hero sentry dart)

foreach(app ${APPS})
  if(NOT app IN_LIST KNOWN_APPS)
    message(FATAL_ERROR "Unknown app '${app}'. Known apps: ${KNOWN_APPS}")
  endif()

  add_executable(${app} ${app}/main.cpp)
  target_link_libraries(${app} PRIVATE tools hardware ${OpenCV_LIBS})

  if(RM_DEBUG)
    # 无 Viewer 时 ${app} 会立即跳过调试输出并返回 0。
    # app 内按相对路径读取 apps/<app>/config.yaml,故在源码根目录运行。
    add_test(NAME ${app}_debug COMMAND ${app})
    set_tests_properties(${app}_debug PROPERTIES
      WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
      TIMEOUT 60)
  endif()
endforeach()

message(STATUS "Building apps: ${APPS}")
