cmake_minimum_required(VERSION 3.16)
project(rm_vision LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

# Which apps to build; edit this list. Options: infantry hero sentry dart
set(APPS infantry)

find_package(OpenCV REQUIRED)

include_directories(${PROJECT_SOURCE_DIR})

add_subdirectory(apps)
