set(KNOWN_APPS infantry hero sentry dart)

foreach(app ${APPS})
  if(NOT app IN_LIST KNOWN_APPS)
    message(FATAL_ERROR "Unknown app '${app}'. Known apps: ${KNOWN_APPS}")
  endif()

  add_executable(${app} ${app}/main.cpp)
  target_link_libraries(${app} PRIVATE ${OpenCV_LIBS})
endforeach()

message(STATUS "Building apps: ${APPS}")
