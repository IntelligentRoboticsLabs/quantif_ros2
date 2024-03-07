cmake_minimum_required(VERSION 3.8)
project(quantif_ros2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_BUILD_TYPE Debug)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(image_transport REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(quantif_ros2_interfaces REQUIRED)
find_package(ament_index_cpp REQUIRED)

find_package(OpenCV REQUIRED)

set(dependencies
  rclcpp
  sensor_msgs
  image_transport
  cv_bridge
  OpenCV
  quantif_ros2_interfaces
  ament_index_cpp
)

include_directories(include)

add_library(${PROJECT_NAME} SHARED
  src/${PROJECT_NAME}/QR2Node.cpp
)
ament_target_dependencies(${PROJECT_NAME} ${dependencies})

add_executable(small_arq_main src/small_arq_main.cpp)
ament_target_dependencies(small_arq_main ${dependencies})
target_link_libraries(small_arq_main ${PROJECT_NAME})

install(TARGETS
  ${PROJECT_NAME}
  small_arq_main
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config tests/configs DESTINATION share/${PROJECT_NAME})



if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)
  add_subdirectory(tests)
endif()

ament_package()
