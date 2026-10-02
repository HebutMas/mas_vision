# 能量机关识别:OpenVINO 五点模型(shenzhenbuff-0624.onnx),推理依赖 OpenVINO Runtime。
find_package(OpenVINO REQUIRED)
if(NOT TARGET openvino::runtime)
  message(FATAL_ERROR "OpenVINO lacks the \"openvino::runtime\" target. Please confirm the version of the Runtime that has been installed.")
endif()

add_library(auto_buff_detection STATIC
  detection/detector.cpp
  models/model.cpp
  models/shenzhenbuff.cpp)
target_link_libraries(auto_buff_detection PUBLIC tools ${OpenCV_LIBS} openvino::runtime)

# 仓库根目录(编译期写死):config 里的相对模型路径按仓库根解析。
target_compile_definitions(auto_buff_detection PRIVATE RM_AUTO_BUFF_ROOT="${PROJECT_SOURCE_DIR}")

# 模型目录编译期写死,方便测试直接定位。
set(RM_AUTO_BUFF_MODELS_DIR "${CMAKE_CURRENT_SOURCE_DIR}/models")

# 单元测试。默认读取 test/ 目录下所有以 "test" 开头的视频并输出标注视频;
# ctest 用 --self-test 只跑配置与模型 smoke,不读视频。
add_executable(auto_buff_detection_test detection/test.cpp)
target_link_libraries(auto_buff_detection_test PRIVATE auto_buff_detection)
target_compile_definitions(auto_buff_detection_test PRIVATE
  RM_AUTO_BUFF_TEST_DIR="${CMAKE_CURRENT_SOURCE_DIR}/test"
  RM_AUTO_BUFF_MODELS_DIR="${RM_AUTO_BUFF_MODELS_DIR}")

add_test(NAME auto_buff_detection COMMAND auto_buff_detection_test --self-test)
set_tests_properties(auto_buff_detection PROPERTIES TIMEOUT 180)
