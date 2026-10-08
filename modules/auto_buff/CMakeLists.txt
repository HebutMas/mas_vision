# 能量机关识别:OpenVINO 五点模型(shenzhenbuff-0624.onnx),推理依赖 OpenVINO Runtime。
find_package(OpenVINO REQUIRED)
if(NOT TARGET openvino::runtime)
  message(FATAL_ERROR "OpenVINO lacks the \"openvino::runtime\" target. Please confirm the version of the Runtime that has been installed.")
endif()

add_library(auto_buff_detection STATIC
  auto_buff.cpp
  detection/detector.cpp
  models/model.cpp
  models/shenzhenbuff.cpp
  debug/visualize.cpp
  detection/refiner.cpp)
target_link_libraries(auto_buff_detection PUBLIC tools ${OpenCV_LIBS} openvino::runtime)

# 仓库根目录(编译期写死):config 里的相对模型路径按仓库根解析。
target_compile_definitions(auto_buff_detection PRIVATE RM_AUTO_BUFF_ROOT="${PROJECT_SOURCE_DIR}")

# 模型目录编译期写死,方便测试直接定位。
set(RM_AUTO_BUFF_MODELS_DIR "${CMAKE_CURRENT_SOURCE_DIR}/models")

# 单元测试:整个 auto_buff 的对外接口都在这里验(不是只有 detection)
add_executable(auto_buff_test test.cpp)
target_link_libraries(auto_buff_test PRIVATE auto_buff_detection)
target_compile_definitions(auto_buff_test PRIVATE
  RM_AUTO_BUFF_MODELS_DIR="${RM_AUTO_BUFF_MODELS_DIR}")

add_test(NAME auto_buff COMMAND auto_buff_test)
set_tests_properties(auto_buff PROPERTIES TIMEOUT 180)
