cmake_minimum_required(VERSION 2.8.3)
project(clhero_hw)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED
  #COMPONENTS
  roscpp
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES epos_functions movement_orders
  CATKIN_DEPENDS roscpp
  #cereal_port
  #std_msgs
  #message_runtime
  #DEPENDS system_lib
)

###########
## Build ##
###########

#link_directories(/home/jorge/WS/socket_ws/src/altitude_sensor/lib)
#link_directories(/usr/lib)

#link_libraries(EposCmd)

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
    include include
    ${catkin_INCLUDE_DIRS} #${modular_lib_pkg_INCLUDE_DIRS}
)

#include_directories(/usr/local/include)
#link_directories(/usr/local/lib)
#link_directories(/usr/lib)
#link_libraries(usb)
#link_libraries(EposCmd.so)


## Declare a cpp library
add_library(movement_orders libraries/movement_orders/movement_orders.cpp)
add_library(epos_functions libraries/epos_functions/epos_functions.cpp)

## Declare a cpp executable
# Poner el robot en pie -> altitude_sensor_node
# Caminar en trípode alterno -> hexapodo_tripode_alterno
# Control desde teclado -> clhero_movent

#add_executable(altitude_sensor_node src/robot_position_stand_up.cpp)
#add_executable(hexapodo_tripode_alterno src/robot_position_control_tripod_2.cpp)
add_executable(clhero_movement src/robot_movement.cpp)

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
#add_dependencies(altitude_sensor_node altitude_sensor_generate_messages_cpp ${catkin_EXPORTED_TARGETS}) #hexapod_class)
#add_dependencies(hexapodo_tripode_alterno altitude_sensor_generate_messages_cpp ${catkin_EXPORTED_TARGETS}) #hexapod_class)
add_dependencies(clhero_movement ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
#target_link_libraries(altitude_sensor_node ${catkin_LIBRARIES})
#target_link_libraries(altitude_sensor_node ${modular_lib_pkg_LIBRARIES})
#target_link_libraries(altitude_sensor_node ${cereal_port_LIBRARIES})
#target_link_libraries(altitude_sensor_node usb-1.0)
#target_link_libraries(altitude_sensor_node EposCmd)
#target_link_libraries(altitude_sensor_node epos_functions)
#target_link_libraries(altitude_sensor_node hexapod_class)

#target_link_libraries(hexapodo_tripode_alterno ${catkin_LIBRARIES})
#target_link_libraries(hexapodo_tripode_alterno ${modular_lib_pkg_LIBRARIES})
#target_link_libraries(altitude_sensor_node ${cereal_port_LIBRARIES})
#target_link_libraries(hexapodo_tripode_alterno usb-1.0)
#target_link_libraries(hexapodo_tripode_alterno EposCmd)
#target_link_libraries(hexapodo_tripode_alterno epos_functions)

target_link_libraries(clhero_movement ${catkin_LIBRARIES})
#target_link_libraries(clhero_movement ${modular_lib_pkg_LIBRARIES})
#target_link_libraries(altitude_sensor_node ${cereal_port_LIBRARIES})
#target_link_libraries(clhero_movement usb-1.0)
#target_link_libraries(clhero_movement EposCmd)
#target_link_libraries(clhero_movement epos_functions)
target_link_libraries(clhero_movement movement_orders)
