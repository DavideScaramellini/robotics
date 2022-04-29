cmake_minimum_required(VERSION 2.8.pub3)
project(main_node)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp std_msgs rosbag message_generation dynamic_reconfigure)

## Add custom message and its dependencies
add_message_files(
        FILES
        wheels.msg
)

add_service_files(
        FILES
        Reset.srv
)

generate_dynamic_reconfigure_options(
        cfg/parameters.cfg
)

generate_messages(
        DEPENDENCIES
        std_msgs
)

## Declare a catkin package
catkin_package(CATKIN_DEPENDS message_runtime)

## Build talker and listener

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(publisher src/pub.cpp)
add_executable(listener src/Movements.cpp)

target_link_libraries(publisher ${catkin_LIBRARIES})
target_link_libraries(listener ${catkin_LIBRARIES})

add_dependencies(publisher ${catkin_EXPORTED_TARGETS})
add_dependencies(listener ${catkin_EXPORTED_TARGETS})
add_dependencies(publisher ${main_node}_gencfg)

