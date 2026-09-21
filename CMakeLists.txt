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

# 远程调试(Rerun):编辑此处开关。
set(RM_DEBUG ON)

find_package(OpenCV REQUIRED)

include_directories(${PROJECT_SOURCE_DIR})

enable_testing()

if(RM_DEBUG)
  include(FetchContent)
  # 默认用仓库内 3rdparty/ 里已随源码携带的 SDK,离线可用;也可覆盖成本地路径或 URL。
  set(RM_RERUN_SDK_ZIP "${PROJECT_SOURCE_DIR}/3rdparty/rerun_cpp_sdk.zip"
      CACHE STRING "Path or URL of rerun_cpp_sdk.zip")

  FetchContent_Declare(rerun_sdk URL ${RM_RERUN_SDK_ZIP})
  FetchContent_MakeAvailable(rerun_sdk)

  # Rerun SDK 默认从 GitHub 拉 Apache Arrow 18.0.0 源码再本机构建。先按校验和把源码包
  # 预置到其 ExternalProject 的下载目录,网络受限时可直接命中镜像、跳过 GitHub;
  # 预置失败也不影响 SDK 自带的 GitHub 回退。
  set(RM_ARROW_TARBALL "${PROJECT_SOURCE_DIR}/3rdparty/apache-arrow-18.0.0.tar.gz"
      CACHE FILEPATH "Local apache-arrow-18.0.0.tar.gz (optional)")
  set(RM_ARROW_URL
      "https://mirrors.huaweicloud.com/apache/arrow/arrow-18.0.0/apache-arrow-18.0.0.tar.gz"
      CACHE STRING "Mirror URL of the Apache Arrow source tarball")
  set(RM_ARROW_MD5 96a4e40287137867c9fe7a2b6c3e5083)

  set(_arrow_dir "${FETCHCONTENT_BASE_DIR}/rerun_sdk-build/arrow/src")
  set(_arrow_dst "${_arrow_dir}/apache-arrow-18.0.0.tar.gz")

  set(_arrow_ok FALSE)
  if(EXISTS "${_arrow_dst}")
    file(MD5 "${_arrow_dst}" _arrow_hash)
    if("${_arrow_hash}" STREQUAL "${RM_ARROW_MD5}")
      set(_arrow_ok TRUE)
    endif()
  endif()

  if(NOT _arrow_ok)
    if(RM_ARROW_TARBALL AND EXISTS "${RM_ARROW_TARBALL}")
      message(STATUS "Pre-seeding Arrow from ${RM_ARROW_TARBALL}")
      file(MAKE_DIRECTORY "${_arrow_dir}")
      file(COPY "${RM_ARROW_TARBALL}" DESTINATION "${_arrow_dir}")
    elseif(RM_ARROW_URL)
      message(STATUS "Pre-seeding Arrow from ${RM_ARROW_URL}")
      file(MAKE_DIRECTORY "${_arrow_dir}")
      file(DOWNLOAD "${RM_ARROW_URL}" "${_arrow_dst}" EXPECTED_MD5 "${RM_ARROW_MD5}"
           TIMEOUT 120 SHOW_PROGRESS STATUS _arrow_status)
    endif()
  endif()

  # Arrow 的精简构建还会从 GitHub 拉一个 xsimd 13.0.0 压缩包。同样按哈希预置,
  # 与 Arrow 的处理方式一致:匹配则跳过下载,缺失/不符则回退到 SDK 自带的 URL。
  set(RM_XSIMD_TARBALL "${PROJECT_SOURCE_DIR}/3rdparty/xsimd-13.0.0.tar.gz"
      CACHE FILEPATH "Local xsimd-13.0.0.tar.gz (optional)")
  set(RM_XSIMD_SHA256 8bdbbad0c3e7afa38d88d0d484d70a1671a1d8aefff03f4223ab2eb6a41110a3)

  set(_xsimd_dir "${FETCHCONTENT_BASE_DIR}/rerun_sdk-build/arrow/src/arrow_cpp-build/src")
  set(_xsimd_dst "${_xsimd_dir}/13.0.0.tar.gz")

  set(_xsimd_ok FALSE)
  if(EXISTS "${_xsimd_dst}")
    file(SHA256 "${_xsimd_dst}" _xsimd_hash)
    if("${_xsimd_hash}" STREQUAL "${RM_XSIMD_SHA256}")
      set(_xsimd_ok TRUE)
    endif()
  endif()

  if(NOT _xsimd_ok AND RM_XSIMD_TARBALL AND EXISTS "${RM_XSIMD_TARBALL}")
    message(STATUS "Pre-seeding xsimd from ${RM_XSIMD_TARBALL}")
    file(MAKE_DIRECTORY "${_xsimd_dir}")
    file(COPY "${RM_XSIMD_TARBALL}" DESTINATION "${_xsimd_dir}")
  endif()
endif()

add_subdirectory(tools)
add_subdirectory(apps)
