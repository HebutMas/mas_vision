set(KNOWN_APPS infantry hero sentry dart)

foreach(app ${APPS})
  if(NOT app IN_LIST KNOWN_APPS)
    message(FATAL_ERROR "Unknown app '${app}'. Known apps: ${KNOWN_APPS}")
  endif()

  add_executable(${app} ${app}/main.cpp)
  target_link_libraries(${app} PRIVATE tools hardware ${OpenCV_LIBS})
  # 配置默认走绝对路径,这样从任意目录运行都能找到 apps/<app>/config.yaml。
  target_compile_definitions(${app} PRIVATE RM_CONFIG_PATH="${CMAKE_SOURCE_DIR}/apps/${app}/config.yaml")
endforeach()

message(STATUS "Building apps: ${APPS}")
