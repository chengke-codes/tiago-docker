cmake_minimum_required(VERSION 2.8.3)
project(simple_models_controller_configuration)
find_package(catkin REQUIRED)
catkin_package()

foreach(dir config)
    install(DIRECTORY ${dir}/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach(dir)
