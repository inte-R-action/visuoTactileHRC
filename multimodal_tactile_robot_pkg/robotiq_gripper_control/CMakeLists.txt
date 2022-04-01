cmake_minimum_required(VERSION 2.8.3)
project(multimodal_tactile_gripper_ctrl)

#find_package(catkin REQUIRED COMPONENTS
#  robotiq_2f_gripper_control
#)

add_executable(${PROJECT_NAME} src/robotiq_gripper_control.cpp)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${Boost_LIBRARIES})
add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})
install(TARGETS ${PROJECT_NAME} DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

add_executable(multimodal_tactile_gripper_ctrl_test src/gripper_demo_test.cpp)
target_link_libraries(multimodal_tactile_gripper_ctrl_test ${catkin_LIBRARIES} ${Boost_LIBRARIES})
add_dependencies(multimodal_tactile_gripper_ctrl_test ${catkin_EXPORTED_TARGETS})
install(TARGETS multimodal_tactile_gripper_ctrl_test DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
