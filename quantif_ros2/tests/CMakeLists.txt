
ament_add_gtest(qr2node_tests_1 qr2node_tests_1.cpp)
ament_target_dependencies(qr2node_tests_1 ${dependencies})
target_link_libraries(qr2node_tests_1 ${PROJECT_NAME})
