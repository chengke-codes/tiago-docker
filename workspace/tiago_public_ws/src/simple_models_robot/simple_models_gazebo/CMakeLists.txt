cmake_minimum_required(VERSION 2.8.3)
project(simple_models_gazebo)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")

find_package(gazebo REQUIRED)
find_package(Boost REQUIRED)
find_package(catkin REQUIRED COMPONENTS
    gazebo_ros
    gazebo_plugins
    )

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES gazebo_diff_drive # gazebo_force_control
  CATKIN_DEPENDS gazebo_ros
  DEPENDS GAZEBO Boost
)

string(REPLACE "-lpthread;" "" GAZEBO_LIBRARIES "${GAZEBO_LIBRARIES}")

include_directories(SYSTEM ${catkin_INCLUDE_DIRS} ${GAZEBO_INCLUDE_DIRS}
    ${SDFormat_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})
include_directories(include)

link_directories(${GAZEBO_LIBRARY_DIRS})

add_library(gazebo_diff_drive src/gazebo_diff_drive.cpp)
target_link_libraries(gazebo_diff_drive ${GAZEBO_LIBRARIES} ${catkin_LIBRARIES})

add_library(gazebo_diff_drive_plugin src/gazebo_diff_drive_plugin.cpp)
target_link_libraries(gazebo_diff_drive_plugin gazebo_diff_drive ${GAZEBO_LIBRARIES} ${catkin_LIBRARIES})

add_library(gazebo_zero_wheel_vel src/gazebo_zero_wheel_vel.cpp)
target_link_libraries(gazebo_zero_wheel_vel ${GAZEBO_LIBRARIES} ${catkin_LIBRARIES})

add_library(gazebo_four_wheel_diff_drive_plugin src/gazebo_four_wheel_diff_drive_plugin.cpp)
target_link_libraries(gazebo_four_wheel_diff_drive_plugin gazebo_diff_drive ${GAZEBO_LIBRARIES} ${catkin_LIBRARIES})

# add_library(gazebo_force_control src/gazebo_force_control.cpp)
# target_link_libraries(gazebo_force_control ${GAZEBO_LIBRARIES} ${catkin_LIBRARIES})

install(TARGETS
    gazebo_diff_drive gazebo_zero_wheel_vel gazebo_diff_drive_plugin
    gazebo_four_wheel_diff_drive_plugin # gazebo_force_control
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    )


install(DIRECTORY include/
        DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION})

foreach(dir launch)
    install(DIRECTORY ${dir}/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach(dir)
