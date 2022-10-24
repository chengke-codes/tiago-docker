cmake_minimum_required(VERSION 2.8.3)
project(joy_teleop)

SET(CMAKE_CXX_FLAGS "-std=c++0x ${CMAKE_CXX_FLAGS}")

find_package(catkin REQUIRED COMPONENTS
    roscpp
    actionlib
    teleop_tools_msgs
    urdf)
catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS actionlib teleop_tools_msgs)

include_directories(SYSTEM ${catkin_INCLUDE_DIRS})
include_directories(include)

add_executable(incrementer_server src/incrementer_server.cpp)
target_link_libraries(incrementer_server ${catkin_LIBRARIES})
add_dependencies(incrementer_server ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

install(PROGRAMS scripts/joy_teleop.py 
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(TARGETS incrementer_server
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(DIRECTORY include/
        DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION})

foreach(dir config launch)
    install(DIRECTORY ${dir}/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach(dir)


