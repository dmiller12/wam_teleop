cmake_minimum_required(VERSION 2.6)
project(wam_teleop)

option(BUILD_LEADER "Build leader executable" ON)

# By default, build programs with compiler optimizations and debugging symbols
if (NOT CMAKE_BUILD_TYPE)
   set(CMAKE_BUILD_TYPE "RelWithDebInfo")
endif()

find_package(catkin REQUIRED COMPONENTS
   roscpp
   std_msgs
   sensor_msgs
   message_generation
)

add_message_files(
   FILES
   GravityTorque.msg
)

generate_messages( DEPENDENCIES std_msgs)

catkin_package(
   # INCLUDE_DIRS include
   # LIBRARIES ${PROJECT_NAME}
   CATKIN_DEPENDS roscpp std_msgs sensor_msgs message_runtime
)


## libbarrett
find_package(Barrett REQUIRED)
include_directories(${BARRETT_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})
link_directories(${BARRETT_LIBRARY_DIRS})
add_definitions(${BARRETT_DEFINITIONS})

find_package(Boost REQUIRED COMPONENTS system thread)

if (BUILD_LEADER) 
   find_package(haptic_wrist 2.0 REQUIRED)
   add_executable(leader src/leader.cpp src/udp_handler.cpp)
   target_link_libraries(leader ${BARRETT_LIBRARIES} ${catkin_LIBRARIES} haptic_wrist Boost::system Boost::thread)
   target_compile_definitions(leader PRIVATE BUILD_LEADER)
   add_dependencies(leader ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
endif()

add_executable(follower src/follower.cpp src/udp_handler.cpp)
target_link_libraries(follower ${BARRETT_LIBRARIES} ${catkin_LIBRARIES} Boost::system Boost::thread)
add_dependencies(follower ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
