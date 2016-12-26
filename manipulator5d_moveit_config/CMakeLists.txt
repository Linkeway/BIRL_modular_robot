cmake_minimum_required(VERSION 2.8.3)
project(manipulator5d_moveit_config)

find_package(catkin REQUIRED)

catkin_package()

install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY config DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

add_executable(planning_scene_ros_api_tutorial src/planning_scene_ros_api_tutorial.cpp)
target_link_libraries(planning_scene_ros_api_tutorial ${catkin_LIBRARIES} ${Boost_LIBRARIES})
install(TARGETS planning_scene_ros_api_tutorial DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
