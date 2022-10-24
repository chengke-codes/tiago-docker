cmake_minimum_required(VERSION 3.0.2)
project(actuator_state_controller)

add_compile_options(-std=c++11)
find_package(catkin REQUIRED COMPONENTS
  controller_interface
  hardware_interface
  pluginlib
  realtime_tools
  sensor_msgs
)

catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES actuator_state_controller
#  CATKIN_DEPENDS controller_interface hardware_interface pluginlib realtime_tools sensor_msgs
#  DEPENDS system_lib
)

include_directories(
 include
)

include_directories(
  SYSTEM
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
add_library(${PROJECT_NAME}
   src/actuator_state_controller.cpp)

target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

#############
## Install ##
#############
# Install
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )

install(FILES actuator_state_plugin.xml
              actuator_state_controller.yaml
              actuator_state_controller.launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

#############
## Test ##
#############
if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)

  add_rostest_gtest(actuator_state_controller_test
                    test/actuator_state_controller.test
                    test/actuator_state_controller_test.cpp)
  target_link_libraries(actuator_state_controller_test ${PROJECT_NAME})
endif()
