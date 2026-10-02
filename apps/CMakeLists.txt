set(KNOWN_APPS infantry hero sentry dart calibration)

foreach(app ${APPS})
  if(NOT app IN_LIST KNOWN_APPS)
    message(FATAL_ERROR "Unknown app '${app}'. Known apps: ${KNOWN_APPS}")
  endif()

  add_executable(${app} ${app}/main.cpp)
  if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/${app}/main_debug.cpp)
    target_sources(${app} PRIVATE ${app}/main_debug.cpp)
  endif()
  target_link_libraries(${app} PRIVATE tools hardware auto_armor_detection auto_buff_detection ${OpenCV_LIBS})
  # 配置默认走绝对路径,这样从任意目录运行都能找到 apps/<app>/config.yaml。
  target_compile_definitions(${app} PRIVATE RM_CONFIG_PATH="${CMAKE_SOURCE_DIR}/apps/${app}/config.yaml")
  # 配置里的相对路径(如标定输出)按仓库根解析,不依赖启动时的工作目录。
  target_compile_definitions(${app} PRIVATE RM_PROJECT_ROOT="${CMAKE_SOURCE_DIR}")
endforeach()

message(STATUS "Building apps: ${APPS}")
