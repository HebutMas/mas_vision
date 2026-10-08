find_package(OpenVINO REQUIRED)
if(NOT TARGET openvino::runtime)
  message(FATAL_ERROR "OpenVINO lacks the \"openvino::runtime\" target. Please confirm the version of the Runtime that has been installed.")
endif()

add_library(auto_armor_detection STATIC
  auto_armor.cpp
  detection/detector.cpp
  detection/green_light.cpp
  debug/visualize.cpp
  models/model.cpp
  models/shenzhen_model.cpp)
target_link_libraries(auto_armor_detection PUBLIC tools ${OpenCV_LIBS} openvino::runtime)

# 仓库根目录(编译期写死):config 里的相对路径(如 detector.model)按仓库根解析,
# 这样从任意工作目录启动都能找到模型。
target_compile_definitions(auto_armor_detection PRIVATE RM_AUTO_ARMOR_ROOT="${PROJECT_SOURCE_DIR}")

# 模型目录编译期写死,方便测试 / bench 直接定位。
set(RM_AUTO_ARMOR_MODELS_DIR "${CMAKE_CURRENT_SOURCE_DIR}/models")

# 单元测试:整个 auto_armor 的对外接口都在这里验(不是只有 detection)
add_executable(auto_armor_test test.cpp)
target_link_libraries(auto_armor_test PRIVATE auto_armor_detection)
target_compile_definitions(auto_armor_test PRIVATE
  RM_AUTO_ARMOR_MODELS_DIR="${RM_AUTO_ARMOR_MODELS_DIR}")
add_test(NAME auto_armor COMMAND auto_armor_test)
set_tests_properties(auto_armor PROPERTIES TIMEOUT 180)
