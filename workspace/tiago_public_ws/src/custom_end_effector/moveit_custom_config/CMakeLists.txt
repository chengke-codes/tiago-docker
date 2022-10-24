cmake_minimum_required(VERSION 2.8.3)
project(moveit_custom_config)

find_package(catkin REQUIRED)

###################################
## catkin specific configuration ##
###################################
catkin_package()

#############
## Install ##
#############

install(DIRECTORY config
	    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
	    USE_SOURCE_PERMISSIONS
)