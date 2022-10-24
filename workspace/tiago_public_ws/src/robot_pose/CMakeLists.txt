cmake_minimum_required(VERSION 2.8.3)
project(robot_pose)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  pal_statistics
  roscpp
  std_msgs
  tf2
  tf2_geometry_msgs
  nav_msgs
)

catkin_package()
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
###########
## Build ##
###########

include_directories(include)
include_directories(SYSTEM ${catkin_INCLUDE_DIRS})

add_executable(robot_pose_node 
  src/pose_publisher.cpp
)

target_link_libraries(robot_pose_node
  ${catkin_LIBRARIES}
)

add_executable(distance_publisher_node 
  src/distance_publisher.cpp
)

target_link_libraries(distance_publisher_node
  ${catkin_LIBRARIES}
)

add_executable(fake_odom_publisher
  src/fake_odom_publisher.cpp
)

target_link_libraries(fake_odom_publisher 
  ${catkin_LIBRARIES}
)


#############
## Install ##
#############

# Mark executables and/or libraries for installation
install(TARGETS robot_pose_node distance_publisher_node fake_odom_publisher
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

if (CATKIN_ENABLE_TESTING)
    find_package(rostest REQUIRED)

    add_rostest_gtest(distance_travelled-test test/distance_travelled.test test/gtest_distance_travelled.cpp)
    target_link_libraries(distance_travelled-test
                          ${catkin_LIBRARIES})

    add_rostest_gtest(robot_pose-test test/robot_pose.test test/gtest_robot_pose.cpp)
    target_link_libraries(robot_pose-test
                          ${catkin_LIBRARIES})

    add_rostest_gtest(fake_odom-test test/fake_odom.test test/gtest_fake_odom.cpp)
    target_link_libraries(fake_odom-test
                          ${catkin_LIBRARIES})


endif(CATKIN_ENABLE_TESTING)
